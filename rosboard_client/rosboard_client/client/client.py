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

    def onClose(self, wasClean, code, reason) -> None:
        """!
        Function run when the connection is closed
        """
        RosboardClientProtocol.is_connected = False
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

        # rosboard messages are list with the following structure: [_identifier_, {_field1_: _value1_, ...}]
        # Identifiers are contained in the WebsocketV1Transport class
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

    def ready(self, proto: WebSocketClientFactory) -> None:
        """!
        Function to set the protocol object in the client. This allows to send messages
        to the server
        @param proto (WebSocketClientFactory) the protocol object
        """
        ReconnectingClientFactory.resetDelay(self)
        self._proto = proto

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
