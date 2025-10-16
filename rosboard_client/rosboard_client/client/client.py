#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
"""
License:
    Rosboard Client: a client for streaming data from a server using rosboard.
    Copyright (C) 2022 Kiwibot, Inc. or its Affiliates, Ai&Robotics
    
    This file is part of Rosboard Client.

    Rosboard Client is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    Rosboard Client is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
    more details.

    You should have received a copy of the GNU General Public License along
    with Rosboard Client. If not, see <https://www.gnu.org/licenses/>.

Code Information:
    Code Information:
    Maintainer: Eng. Pedro Alejandro Gonzalez B
	Mail: pedro@kiwibot.com
"""
# =============================================================================

import json
import os
import logging
import threading
import time

from autobahn.twisted.websocket import (
    WebSocketClientFactory,
    WebSocketClientProtocol,
    connectWS,
)
from rosboard_client.client.decoders import RosboardDecoder
from twisted.internet import reactor
from twisted.internet.error import ReactorAlreadyRunning, ReactorNotRunning
from twisted.internet.protocol import ReconnectingClientFactory
from twisted.internet.task import LoopingCall

logging.basicConfig(level=logging.INFO)


class WebsocketV1Transport:
    """!
    Class containing the identifying character for each type of rosboard message
    # rosboard messages are list with the following structure [_identifier_, {_field1_: _value1_, ...}]
    where _identifier_ can be any character contained in this class
    """

    MSG_PING = "p"
    MSG_PONG = "q"
    MSG_MSG = "m"
    MSG_TOPICS = "t"
    MSG_SUB = "s"
    MSG_SYSTEM = "y"
    MSG_UNSUB = "u"
    MSG_UNPUB = "n"
    PING_SEQ = "s"
    PONG_SEQ = "s"
    PONG_TIME = "t"


class RosboardClientProtocol(WebSocketClientProtocol):
    """! Class specifying the rosboard client websocket protocol.
    Inherits from the WebSocketClientProtocol from twisted
    """

    # Define a flag to indicate the protocol status
    is_connected = False

    def onConnect(self, response) -> None:
        """!
        Function run when the websocket is successfully connected
        @param response the connection response
        """
        RosboardClientProtocol.is_connected = True
        self.factory.logger.info(f"Server connected: {response.peer}")

    def onOpen(self) -> None:
        """!
        Function run when the connection is open. This sets
        the protocol object in the factory using this class
        """
        RosboardClientProtocol.is_connected = True
        self.factory.logger.info(f"Communication opened")
        self.factory.ready(self)
        # Track connection open time and increment reconnect counter
        try:
            self.factory._open_time = time.time()
            self.factory._reconnect_count += 1
            self.factory.logger.info(
                f"Conn opened: reconnects={self.factory._reconnect_count}"
            )
            # Start periodic health logging (every 60s)
            self.factory.start_health_logging(60.0)
        except Exception:
            pass

    def onClose(self, wasClean, code, reason) -> None:
        """!
        Function run when the connection is closed
        """
        RosboardClientProtocol.is_connected = False
        # Log extended connection stats to help detect patterns
        try:
            now = time.time()
            open_time = getattr(self.factory, "_open_time", None)
            last_rx_time = getattr(self.factory, "_last_rx_time", None)
            last_tx_time = getattr(self.factory, "_last_tx_time", None)
            conn_age = (now - open_time) if open_time else None
            last_rx_age = (now - last_rx_time) if last_rx_time else None
            last_tx_age = (now - last_tx_time) if last_tx_time else None
            subs_count = len(getattr(self.factory, "socket_subscriptions", {}))
            # Store event snapshot
            event = {
                "ts": now,
                "wasClean": wasClean,
                "code": code,
                "reason": f"{reason}",
                "conn_age_s": conn_age,
                "last_rx_age_s": last_rx_age,
                "last_tx_age_s": last_tx_age,
                "subs": subs_count,
            }
            getattr(self.factory, "_close_events", []).append(event)
            self.factory.logger.warning(
                f"Communication closed. reason: {reason} was clean: {wasClean}, code: {code}"
            )
            self.factory.logger.info(
                f"Conn stats at close: age={conn_age:.1f}s rx_age={last_rx_age:.1f}s tx_age={last_tx_age:.1f}s subs={subs_count}"
            )
        except Exception:
            # Fallback to original log if any formatting fails
            self.factory.logger.warning(
                f"Communication closed. reason: {reason} was clean: {wasClean}, code: {code}"
            )

    def onMessage(self, payload, isBinary) -> None:
        """!
        Function run each time a message is received. A callback function
        is called if a ROS message is received and topics are stored in case
        they are received.
        @param payload the content of the message
        @param isBinary (bool) _description_
        """
        if not isBinary:
            data = json.loads(payload.decode("utf8"))
            # Track last RX time
            try:
                self.factory._last_rx_time = time.time()
            except Exception:
                pass

        # rosboard messages are list with the following structure: [_identifier_, {_field1_: _value1_, ...}]
        # Identifiers are contained in the WebsocketV1Transport class
        # Respond to server ping to avoid server-side session timeouts
        if data[0] == WebsocketV1Transport.MSG_PING:
            try:
                now_ms = int(time.time() * 1000)
                seq = 0
                if isinstance(data[1], dict) and WebsocketV1Transport.PING_SEQ in data[1]:
                    seq = data[1][WebsocketV1Transport.PING_SEQ]
                pong = json.dumps(
                    [
                        WebsocketV1Transport.MSG_PONG,
                        {
                            WebsocketV1Transport.PONG_SEQ: seq,
                            WebsocketV1Transport.PONG_TIME: now_ms,
                        },
                    ]
                ).encode("utf-8")
                # We are already in reactor thread
                self.sendMessage(pong, isBinary=False)
                try:
                    self.factory._last_tx_time = time.time()
                except Exception:
                    pass
            except Exception as e:
                try:
                    self.factory.logger.debug(f"Failed to send PONG: {e}")
                except Exception:
                    pass
            return

        # In case the information received contains a ros message
        if data[0] == WebsocketV1Transport.MSG_MSG:
            data = RosboardDecoder.decode_binary_fields(data)
            topic_name = data[1]["_topic_name"]
            if topic_name in self.factory.socket_subscriptions:
                self.factory.socket_subscriptions[topic_name](data)
            # print(f"got message on topic {data[1]}")
        # in case the information received contains the list of available topics
        if data[0] == WebsocketV1Transport.MSG_TOPICS:
            self.factory.set_available_topics(data[1])

    def send_message(self, payload) -> None:
        """! Function to send a message to the server. Is called on a separate thread
        @param payload (json-like) a json-like object with the payload
        """
        return reactor.callFromThread(
            self.sendMessage,
            payload,
            isBinary=False,
            fragmentSize=None,
            sync=False,
            doNotCompress=False,
        )


class RosboardClient(ReconnectingClientFactory, WebSocketClientFactory):

    protocol = RosboardClientProtocol

    # Specify the max time between connection attempts on reconnection
    maxDelay = 2.0

    def __init__(self, host: str, connection_timeout: float):
        """! Class containing the socket client to connect to the rosboard server
        It inherits from twisted's WebSocketClientFactory
        @param host (str) the address of the rosboard server. ex: '127.0.0.1:8888'
        @param connection_timeout (float) the time to wait for connecting and getting the available topics
        Raises:
            Exception: In case connection was not established within the timeout
            Exception: In case the server did not send the available topic within the timeout
        """
        self.logger = logging.getLogger("rosboard_client")
        self.is_connected = False
        self.socket_subscriptions = {}
        self.available_topics = {}
        # Telemetry for debugging connection patterns
        self._open_time = None
        self._last_rx_time = None
        self._last_tx_time = None
        self._reconnect_count = -1  # will become 0 on first open
        self._close_events = []
        self._health_log = None
        self._app_keepalive = None
        self._app_keepalive_interval = float(
            os.getenv("WS_APP_KEEPALIVE_S", "25")
        )
        self._age_guard = None
        self._age_guard_interval = float(os.getenv("WS_AGE_GUARD_S", "30"))
        # Refresh before TTL: reconnect proactively if connection age exceeds threshold
        # Default: 300s (5min) minus a safety margin of 30s
        self._age_guard_ttl_s = float(os.getenv("WS_TTL_S", "300"))
        self._age_guard_margin_s = float(os.getenv("WS_TTL_MARGIN_S", "30"))

        # Logging toggles for health and keepalive diagnostics
        # Defaults: disabled (set to 1 to enable)
        try:
            self._log_health_enabled = int(os.getenv("WS_LOG_HEALTH", "0")) == 1
        except Exception:
            self._log_health_enabled = True
        try:
            self._log_keepalive_enabled = int(os.getenv("WS_LOG_KEEPALIVE", "0")) == 1
        except Exception:
            self._log_keepalive_enabled = True
        # Optional global override for both toggles
        _global_diag = os.getenv("WS_LOG_CONN_DIAG")
        if _global_diag is not None:
            try:
                _enabled = int(_global_diag) == 1
                self._log_health_enabled = _enabled
                self._log_keepalive_enabled = _enabled
            except Exception:
                pass

        # Define the socket URL
        if host.startswith("ws://"):
            socket_url = host + "/rosboard/v1"

        elif host.startswith("wss://"):
            socket_url = host + "/rosboard/v1"
            self.isSecure = True

        else:
            self.logger.info(
                "Websocket protocol not provided in host url. Falling back to ws:// as default"
            )

            # Get the port from host
            try:
                # Get last part of string after split
                port = int(host.split(":")[-1])

            # If value can not be parsed to int, use port 80
            except ValueError:
                port = 80
                self.logger.warn(
                    "Could not parse port passed to rosboard client. Using port 80 as default."
                )

            # Add generic exception to prevent errors
            except Exception:
                port = 80
                self.logger.warn(
                    "There was an error while trying to get port. Using port 80 as default."
                )

            # Use corresponding protocol to connect to server
            if port == 443:
                socket_url = "wss://" + host + "/rosboard/v1"
                self.isSecure = True
            else:
                socket_url = "ws://" + host + "/rosboard/v1"

        WebSocketClientFactory.__init__(self, url=socket_url)
        self.logger.info(f"connecting to {socket_url}")
        # Enable WebSocket keepalive to prevent idle session timeouts in proxies/tunnels
        # Sends a ping every 30s and expects a pong within 10s
        try:
            self.setProtocolOptions(
                autoPingInterval=30,
                autoPingTimeout=10,
                autoPingPayload=b"rb",
            )
        except Exception:
            # If the underlying Autobahn version does not support these options,
            # proceed without failing hard; reconnection logic will still work
            pass
        self.connector = connectWS(self, timeout=connection_timeout)

        # protocol object. Set when the socket is ready
        self._proto = None

        # Run the reactor in a separate thread
        self._thread = threading.Thread(target=self.run_reactor)
        self._thread.daemon = True
        self._thread.start()

        # Check if connection takes more than the timeout
        connection_request_time = time.time()
        while self._proto is None:
            time.sleep(0.05)
            self.is_connected = True
            if time.time() - connection_request_time > connection_timeout:
                self.logger.error(f"Connection attempt to {socket_url} timed out")
                ReconnectingClientFactory.stopTrying(self)
                raise Exception("Connection timed out")

        # Check if rosboard returns the topics available on the server within the timeout
        while not self.available_topics:
            time.sleep(0.05)
            if time.time() - connection_request_time > connection_timeout:
                self.logger.error(
                    f"{socket_url} did not send the available topics. Timed out"
                )
                raise Exception("Available topics not received")

        self.logger.info("available topics advertised by server")

    def run_reactor(self):
        """! Function to start the reactor. Handles if the reactor is already running."""
        try:
            reactor.run(False)
        except ReactorAlreadyRunning as e:
            self.logger.warning("Reactor not started as its already running.")

    def stop_reactor(self):
        """! Function to stop the reactor. Handles the error if the reactor is not running."""
        try:
            reactor.stop()
        except ReactorNotRunning as e:
            self.logger.warning("Reactor not stopped as it was not running.")

    def start_health_logging(self, interval_s: float = 60.0) -> None:
        """Start periodic connection health logging.

        @param interval_s float seconds between health logs
        """
        try:
            if not self._log_health_enabled:
                return
            if self._health_log is None:
                self._health_log = LoopingCall(self._log_connection_health)
            if not self._health_log.running:
                self._health_log.start(interval_s, now=False)
                if self._log_health_enabled:
                    self.logger.info(
                        f"Conn health logging started interval={interval_s}s"
                    )
        except Exception as e:
            self.logger.warning(f"Could not start conn health logging: {e}")

    def _should_refresh_for_age(self) -> bool:
        try:
            if self._open_time is None:
                return False
            conn_age = time.time() - self._open_time
            return conn_age >= max(0.0, self._age_guard_ttl_s - self._age_guard_margin_s)
        except Exception:
            return False

    def _age_guard_check(self) -> None:
        """Check connection age and proactively refresh if near TTL."""
        try:
            if self._proto is None:
                return
            if self._should_refresh_for_age():
                self.logger.info("Age guard: refreshing WebSocket connection before TTL")
                try:
                    # Close with normal code; reconnection factory will reopen
                    self._proto.sendClose(code=1000, reason=b"refresh")
                except Exception:
                    pass
        except Exception as e:
            self.logger.debug(f"Age guard check failed (ignored): {e}")

    def start_age_guard(self, interval_s: float = None) -> None:
        try:
            interval = interval_s or self._age_guard_interval
            if self._age_guard is None:
                self._age_guard = LoopingCall(self._age_guard_check)
            if not self._age_guard.running:
                self._age_guard.start(interval, now=False)
                self.logger.info(f"Age guard started interval={interval}s")
        except Exception as e:
            self.logger.warning(f"Could not start age guard: {e}")

    def stop_age_guard(self) -> None:
        try:
            if self._age_guard is not None and self._age_guard.running:
                self._age_guard.stop()
                self.logger.info("Age guard stopped")
        except Exception as e:
            self.logger.warning(f"Could not stop age guard: {e}")

    def _log_connection_health(self) -> None:
        """Log current connection age and last RX/TX ages to help detect patterns."""
        try:
            now = time.time()
            open_time = self._open_time
            conn_age = (now - open_time) if open_time else None
            last_rx_age = (now - self._last_rx_time) if self._last_rx_time else None
            last_tx_age = (now - self._last_tx_time) if self._last_tx_time else None
            subs_count = len(self.socket_subscriptions)

            conn_age_s = f"{conn_age:.1f}s" if conn_age is not None else "NA"
            rx_age_s = f"{last_rx_age:.1f}s" if last_rx_age is not None else "NA"
            tx_age_s = f"{last_tx_age:.1f}s" if last_tx_age is not None else "NA"

            if self._log_health_enabled:
                self.logger.info(
                    f"Conn health: age={conn_age_s} rx_age={rx_age_s} tx_age={tx_age_s} subs={subs_count}"
                )
        except Exception as e:
            self.logger.warning(f"Conn health log failed: {e}")

    def _send_app_ping(self) -> None:
        """Send an application-level ping to keep proxies/tunnels from idling out.

        Uses the rosboard wire format: ["p", {"s": <seq>, "t": <epoch_ms>}].
        Safe to call even if protocol is not ready.
        """
        try:
            if self._proto is None:
                return
            now_ms = int(time.time() * 1000)
            payload = json.dumps(
                [
                    WebsocketV1Transport.MSG_PING,
                    {WebsocketV1Transport.PING_SEQ: 0, WebsocketV1Transport.PONG_TIME: now_ms},
                ]
            ).encode("utf-8")
            self._proto.send_message(payload)
            self._last_tx_time = time.time()
        except Exception as e:
            if self._log_keepalive_enabled:
                try:
                    self.logger.debug(f"App ping failed (ignored): {e}")
                except Exception:
                    pass

    def start_app_keepalive(self, interval_s: float = None) -> None:
        """Start periodic application-level keepalive pings."""
        try:
            interval = interval_s or self._app_keepalive_interval
            if self._app_keepalive is None:
                self._app_keepalive = LoopingCall(self._send_app_ping)
            if not self._app_keepalive.running:
                self._app_keepalive.start(interval, now=False)
                if self._log_keepalive_enabled:
                    self.logger.info(f"App keepalive started interval={interval}s")
        except Exception as e:
            self.logger.warning(f"Could not start app keepalive: {e}")

    def stop_app_keepalive(self) -> None:
        """Stop periodic application-level keepalive pings."""
        try:
            if self._app_keepalive is not None and self._app_keepalive.running:
                self._app_keepalive.stop()
                if self._log_keepalive_enabled:
                    self.logger.info("App keepalive stopped")
        except Exception as e:
            self.logger.warning(f"Could not stop app keepalive: {e}")

    def create_socket_subscription(self, msg_type: str, topic: str, callback) -> None:
        """! Function to subscribe to a topic available in the rosboard server
        @param msg_type (str) rosboard like message type. Ex: nav_msgs.msg.Path
        @param topic (str) the name of the topic
        @param callback (function) the callback to execute when a message on the topic is received
        """
        self.logger.info(f"creating subscriber for topic {topic}")
        self.socket_subscriptions[topic] = callback
        self._proto.send_message(
            # rosboard expects a message like this" ["s", {topicName: xxx}] to create the subscription
            json.dumps([WebsocketV1Transport.MSG_SUB, {"topicName": topic}]).encode(
                "utf-8"
            ),
        )
        # Track last TX time
        try:
            self._last_tx_time = time.time()
        except Exception:
            pass

    def destroy_socket_subscription(self, topic_name: str):
        """! Function to unsubscribe to a topic
        @param msg_type (str) rosboard like message type. Ex: nav_msgs.msg.Path
        @param topic (str) the name of the topic
        @param callback (function) the callback to execute when a message on the topic is received
        """
        self.logger.info(f"Destroying subscriber for topic {topic_name}")
        if topic_name in self.socket_subscriptions.keys():
            # remove the subscription from the dictionary
            self.socket_subscriptions.pop(topic_name)
        else:
            self.logger.warning(
                f"No subscription had been registered for topic {topic_name}"
            )
        self._proto.send_message(
            # rosboard expects a message like this" ["u", {topicName: xxx}] to destroy the subscription
            json.dumps(
                [WebsocketV1Transport.MSG_UNSUB, {"topicName": topic_name}]
            ).encode("utf-8"),
        )
        try:
            self._last_tx_time = time.time()
        except Exception:
            pass

    def destroy_socket_publisher(self, topic_name: str):
        """! Function to destroy publisher of a topic in server.
        @param topic_name "str" name of the topic publisher that will be destroyed.
        """
        self.logger.info(f"Destroying publisher for topic {topic_name}")
        # Send message to destroy publisher in server. Message is expected to be: ["n", {topicName: xxxx}]
        self._proto.send_message(
            #
            json.dumps(
                [WebsocketV1Transport().MSG_UNPUB, {"topicName": topic_name}]
            ).encode("utf-8"),
        )
        try:
            self._last_tx_time = time.time()
        except Exception:
            pass

    def send_ros_message(self, ros_message_dict: dict) -> None:
        """!
        Function to send a ros message to a rosboard server. The message needs to be
        already in a dict form. It must also contain the _topic_name and _topic_type fields
        i.e: {_topic_name: /chatter, _topic_type: std_msgs/msg/String, _data:'Hi!'}
        @param ros_message_dict (dict) the ros message as a dictionary. It must also contain the
        _topic_name and _topic_type fields i.e: {_topic_name: /chatter, _topic_type: std_msgs/msg/String, _data:'Hi!'}
        """
        self._proto.send_message(
            # rosboard expects a message like this" ["m", {message dictionary}] to create the subscription
            json.dumps([WebsocketV1Transport.MSG_MSG, ros_message_dict]).encode(
                "utf-8"
            ),
        )
        try:
            self._last_tx_time = time.time()
        except Exception:
            pass

    def ready(self, proto: WebSocketClientFactory) -> None:
        """!
        Function to set the protocol object in the client. This allows to send messages
        to the server
        @param proto (WebSocketClientFactory) the protocol object
        """
        ReconnectingClientFactory.resetDelay(self)
        self._proto = proto
        # Start application-level keepalive once socket is ready
        self.start_app_keepalive()
        # Start age guard to refresh before TTLs
        self.start_age_guard()

    def set_available_topics(self, topics: dict) -> None:
        """! Function to set the available topics in the client
        @param topics (dict) The dictionary containing all the topics as keys and types as values
        """
        self.available_topics = topics

    def get_available_topics(self) -> list:
        """
        Function to get all the topics available on the server
        @return list with the topics available on the server
        """
        return list(self.available_topics.keys())

    def is_topic_available(self, topic: str) -> bool:
        """!
        Function to check if a given topic is available in the server
        @param topic (str) The topic name
        @return bool whether the topic is available on the server
        """
        return topic in self.available_topics.keys()

    def get_topic_type(self, topic: str) -> str:
        """! Function to get the type of a given topic
        @param topic (str) The topic name
        Raises:
            Exception: if the requested topic is not available in the server
        @return str the topic type using rosboard format. Ex: nav_msgs/msg/Path
        """
        if self.is_topic_available(topic):
            return self.available_topics[topic]
        else:
            self.logger.error(
                f"Unable request type of topic that is not available: {topic}"
            )
            raise Exception("Cannot request type of topic that is not available")

    def clientConnectionLost(self, connector, reason):
        """!
        Function executed when the connection to the server is lost
        """
        self.is_connected = False
        self.logger.error(f"Lost connection with {self.url}, reason: {reason}")
        ReconnectingClientFactory.clientConnectionLost(self, connector, reason)

    def clientConnectionFailed(self, connector, reason):
        """!
        Function executed when the client cannot establish a connection with the server
        """
        self.is_connected = False
        self.logger.error(f"Failed to connect to {self.url}, reason: {reason}")
        ReconnectingClientFactory.clientConnectionFailed(self, connector, reason)
