#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rosboard websocket client.

The original client treated Twisted's process-wide reactor as if it belonged to
each connection.  It also ran ROS decoding and callbacks in the reactor thread.
Both behaviours make a reconnect unreliable: a slow image conversion can starve
the websocket heartbeat, and stopping one client makes it impossible to create
another one without restarting the process.

This implementation gives each ``RosboardClient`` an explicit lifecycle while
sharing one reactor thread for the lifetime of the application.
"""

import json
import logging
import os
import threading
import time
from collections import deque
from typing import Callable, Dict, Optional

from autobahn.twisted.websocket import (
    WebSocketClientFactory,
    WebSocketClientProtocol,
    connectWS,
)
from rosboard_client.client.decoders import RosboardDecoder
from twisted.internet import reactor
from twisted.internet.protocol import ReconnectingClientFactory
from twisted.internet.task import LoopingCall


LOGGER = logging.getLogger("rosboard_client")


class RosboardConnectionError(Exception):
    """Base error for rosboard connection lifecycle failures."""


class RosboardConnectionTimeout(RosboardConnectionError):
    """Raised when the websocket did not become ready before the deadline."""


class RosboardTopicsTimeout(RosboardConnectionError):
    """Raised when the server did not advertise its topic list in time."""


class WebsocketV1Transport:
    """Identifiers used by rosboard's version 1 websocket protocol."""

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


class _ReactorRunner:
    """Start Twisted's global reactor once and keep it alive between clients."""

    _lock = threading.Lock()
    _started = threading.Event()
    _thread = None

    @classmethod
    def ensure_started(cls, timeout: float = 2.0) -> None:
        if reactor.running:
            cls._started.set()
            return

        with cls._lock:
            if reactor.running:
                cls._started.set()
            elif cls._thread is None or not cls._thread.is_alive():
                # A stopped Twisted reactor cannot be restarted.  This should
                # only happen if legacy code called reactor.stop().
                if getattr(reactor, "_startedBefore", False):
                    raise RosboardConnectionError(
                        "The Twisted reactor was stopped and cannot be restarted"
                    )
                cls._started.clear()
                cls._thread = threading.Thread(
                    target=cls._run,
                    name="rosboard-reactor",
                    daemon=True,
                )
                cls._thread.start()

        if not cls._started.wait(timeout):
            raise RosboardConnectionError("Twisted reactor did not start")

    @classmethod
    def _run(cls) -> None:
        reactor.callWhenRunning(cls._started.set)
        reactor.run(installSignalHandlers=False)

    @classmethod
    def stop_for_process_exit(cls) -> None:
        """Stop the shared reactor only when the whole process is exiting."""
        if reactor.running:
            reactor.callFromThread(reactor.stop)


class _LatestMessageDispatcher:
    """Bounded, latest-value callback dispatcher.

    A websocket must never wait for ROS conversion or publication.  There is at
    most one active callback and one pending value per topic; if a producer is
    faster than its consumer, obsolete intermediate values are replaced by the
    newest one instead of building an unbounded latency queue.
    """

    def __init__(self, callback: Callable[[str, list], None], workers: int) -> None:
        self._callback = callback
        self._condition = threading.Condition()
        self._pending = {}
        self._scheduled = set()
        self._ready = deque()
        self._stopping = False
        self.dropped_messages = 0
        self._threads = []

        for index in range(max(1, workers)):
            thread = threading.Thread(
                target=self._run,
                name="rosboard-callback-{}".format(index + 1),
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)

    def submit(self, topic: str, message: list) -> bool:
        with self._condition:
            if self._stopping:
                return False
            if topic in self._pending:
                self.dropped_messages += 1
            self._pending[topic] = message
            if topic not in self._scheduled:
                self._scheduled.add(topic)
                self._ready.append(topic)
                self._condition.notify()
            return True

    def stop(self) -> None:
        with self._condition:
            self._stopping = True
            self._pending.clear()
            self._ready.clear()
            self._condition.notify_all()

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._ready and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                topic = self._ready.popleft()
                message = self._pending.pop(topic)

            try:
                self._callback(topic, message)
            except Exception:
                LOGGER.exception("Unhandled callback error for topic %s", topic)

            with self._condition:
                if self._stopping:
                    return
                if topic in self._pending:
                    self._ready.append(topic)
                    self._condition.notify()
                else:
                    self._scheduled.discard(topic)


class RosboardClientProtocol(WebSocketClientProtocol):
    """Autobahn protocol adapter for :class:`RosboardClient`."""

    def onConnect(self, response) -> None:
        self.factory.logger.info("Server connected: %s", response.peer)

    def onOpen(self) -> None:
        self.is_connected = True
        self.factory._protocol_opened(self)

    def onClose(self, wasClean, code, reason) -> None:
        self.is_connected = False
        self.factory._protocol_closed(self, wasClean, code, reason)

    def onMessage(self, payload, isBinary) -> None:
        received_at = time.monotonic()
        self.factory._record_rx(received_at)

        if isBinary:
            self.factory.logger.warning("Ignoring unexpected binary websocket frame")
            return

        try:
            data = json.loads(payload.decode("utf-8"))
            if not isinstance(data, list) or len(data) < 2:
                raise ValueError("message must be a two-element list")
        except Exception as error:
            self.factory.logger.warning("Invalid rosboard frame: %s", error)
            return

        identifier = data[0]
        if identifier == WebsocketV1Transport.MSG_PING:
            self._reply_to_ping(data[1])
            return

        if identifier == WebsocketV1Transport.MSG_MSG:
            body = data[1]
            if not isinstance(body, dict) or "_topic_name" not in body:
                self.factory.logger.warning("ROS frame has no _topic_name")
                return
            self.factory._message_received(body["_topic_name"], data, received_at)
            return

        if identifier == WebsocketV1Transport.MSG_TOPICS:
            if isinstance(data[1], dict):
                self.factory.set_available_topics(data[1])
            else:
                self.factory.logger.warning("Invalid topic advertisement")

    def _reply_to_ping(self, body) -> None:
        sequence = body.get(WebsocketV1Transport.PING_SEQ, 0) if isinstance(body, dict) else 0
        payload = json.dumps(
            [
                WebsocketV1Transport.MSG_PONG,
                {
                    WebsocketV1Transport.PONG_SEQ: sequence,
                    WebsocketV1Transport.PONG_TIME: int(time.time() * 1000),
                },
            ]
        ).encode("utf-8")
        try:
            self.sendMessage(payload, isBinary=False)
            self.factory._record_tx(time.monotonic())
        except Exception:
            self.factory.logger.exception("Could not reply to rosboard ping")


class RosboardClient(ReconnectingClientFactory, WebSocketClientFactory):
    """Thread-safe rosboard client with reconnect and deterministic shutdown."""

    protocol = RosboardClientProtocol
    initialDelay = 0.25
    factor = 1.7
    maxDelay = 5.0
    jitter = 0.15

    def __init__(self, host: str, connection_timeout: float = 5.0):
        self.logger = LOGGER
        self._state_lock = threading.RLock()
        self._connected_event = threading.Event()
        self._topics_event = threading.Event()
        self._closed_event = threading.Event()
        self._closing = False
        self._proto = None
        self.connector = None
        self.is_connected = False
        self.socket_subscriptions = {}  # type: Dict[str, Callable]
        self.available_topics = {}  # type: Dict[str, str]
        self.connection_generation = 0
        # Kept as a compatibility alias for older AMT diagnostics.
        self._reconnect_count = -1
        self._open_time = None
        self._last_rx_time = None
        self._last_tx_time = None
        self._last_message_time = None
        self._last_topic_message = {}
        self._last_connection_error = None
        self._close_events = deque(maxlen=50)
        self._health_log = None
        self._last_forced_reconnect = 0.0

        try:
            callback_workers = int(os.getenv("ROSBOARD_CALLBACK_WORKERS", "3"))
        except ValueError:
            callback_workers = 3
        self._dispatcher = _LatestMessageDispatcher(
            self._deliver_message, callback_workers
        )

        socket_url = self._build_socket_url(host)
        WebSocketClientFactory.__init__(self, url=socket_url)
        self.logger.info("Connecting to %s", socket_url)

        try:
            self.setProtocolOptions(
                autoPingInterval=self._env_float("WS_PING_INTERVAL_S", 20.0),
                autoPingTimeout=self._env_float("WS_PING_TIMEOUT_S", 8.0),
                # Autobahn 17 uses autoPingSize rather than autoPingPayload.
                autoPingSize=8,
                openHandshakeTimeout=connection_timeout,
                closeHandshakeTimeout=2.0,
            )
        except (AttributeError, TypeError):
            # Autobahn 17 accepts fewer protocol options on some distributions.
            self.setProtocolOptions(
                autoPingInterval=self._env_float("WS_PING_INTERVAL_S", 20.0),
                autoPingTimeout=self._env_float("WS_PING_TIMEOUT_S", 8.0),
                autoPingSize=8,
            )

        try:
            _ReactorRunner.ensure_started()
            reactor.callFromThread(self._connect, connection_timeout)
            deadline = time.monotonic() + connection_timeout
            if not self._connected_event.wait(connection_timeout):
                raise RosboardConnectionTimeout(
                    "Connection to {} timed out{}".format(
                        socket_url,
                        (
                            ": {}".format(self._last_connection_error)
                            if self._last_connection_error
                            else ""
                        ),
                    )
                )

            remaining = max(0.0, deadline - time.monotonic())
            if not self._topics_event.wait(remaining):
                raise RosboardTopicsTimeout(
                    "{} did not advertise topics before timeout".format(socket_url)
                )
        except Exception:
            self.close(timeout=1.0)
            raise

        self.logger.info("Available topics advertised by server")

    @staticmethod
    def _env_float(name: str, default: float) -> float:
        try:
            return float(os.getenv(name, str(default)))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _build_socket_url(host: str) -> str:
        value = host.strip().rstrip("/")
        if not value:
            raise ValueError("Rosboard host cannot be empty")
        if value.startswith("https://"):
            value = "wss://" + value[len("https://") :]
        elif value.startswith("http://"):
            value = "ws://" + value[len("http://") :]
        if not value.startswith(("ws://", "wss://")):
            try:
                port = int(value.rsplit(":", 1)[-1])
            except (ValueError, IndexError):
                port = 80
            value = ("wss://" if port == 443 else "ws://") + value
        if not value.endswith("/rosboard/v1"):
            value += "/rosboard/v1"
        return value

    def _connect(self, connection_timeout: float) -> None:
        if self._closing:
            return
        try:
            self.connector = connectWS(self, timeout=connection_timeout)
        except Exception:
            self.logger.exception("Could not start websocket connector")
            self._closed_event.set()

    def _protocol_opened(self, proto: RosboardClientProtocol) -> None:
        with self._state_lock:
            if self._closing:
                proto.sendClose(code=1000, reason=b"client closing")
                return
            ReconnectingClientFactory.resetDelay(self)
            self._proto = proto
            self.is_connected = True
            self.connection_generation += 1
            self._reconnect_count = self.connection_generation - 1
            self._open_time = time.monotonic()
            self._last_rx_time = self._open_time
            self._last_connection_error = None
            self.available_topics = {}
            self._topics_event.clear()
            subscriptions = list(self.socket_subscriptions)
            self._connected_event.set()

        self.logger.info(
            "Communication opened (generation=%d, subscriptions=%d)",
            self.connection_generation,
            len(subscriptions),
        )
        for topic in subscriptions:
            self._send_subscription(topic)
        self._start_health_logging()

    def _protocol_closed(self, proto, was_clean, code, reason) -> None:
        now = time.monotonic()
        with self._state_lock:
            if self._proto is proto:
                self._proto = None
            self.is_connected = False
            self._connected_event.clear()
            event = {
                "time": time.time(),
                "clean": bool(was_clean),
                "code": code,
                "reason": str(reason),
                "connection_age_s": self._age(now, self._open_time),
                "last_rx_age_s": self._age(now, self._last_rx_time),
                "last_message_age_s": self._age(now, self._last_message_time),
            }
            self._close_events.append(event)
            if not was_clean and not self._closing:
                self._last_connection_error = str(reason)
            closing = self._closing
            if closing:
                self._closed_event.set()

        level = logging.INFO if closing or was_clean else logging.WARNING
        self.logger.log(
            level,
            "Communication closed (clean=%s, code=%s, reason=%s)",
            was_clean,
            code,
            reason,
        )

    @staticmethod
    def _age(now: float, timestamp: Optional[float]) -> Optional[float]:
        return None if timestamp is None else max(0.0, now - timestamp)

    def _record_rx(self, timestamp: float) -> None:
        with self._state_lock:
            self._last_rx_time = timestamp

    def _record_tx(self, timestamp: float) -> None:
        with self._state_lock:
            self._last_tx_time = timestamp

    def _message_received(self, topic: str, data: list, timestamp: float) -> None:
        with self._state_lock:
            self._last_message_time = timestamp
            self._last_topic_message[topic] = timestamp
        self._dispatcher.submit(topic, data)

    def _deliver_message(self, topic: str, data: list) -> None:
        with self._state_lock:
            callback = self.socket_subscriptions.get(topic)
            closing = self._closing
        if callback is None or closing:
            return
        try:
            decoded = RosboardDecoder.decode_binary_fields(data)
            callback(decoded)
        except Exception:
            self.logger.exception("Failed to process rosboard message on %s", topic)

    def _send_payload(self, payload: bytes) -> bool:
        with self._state_lock:
            proto = self._proto
            connected = self.is_connected
            closing = self._closing
        if proto is None or not connected or closing:
            return False
        try:
            proto.sendMessage(payload, isBinary=False)
            self._record_tx(time.monotonic())
            return True
        except Exception:
            self.logger.exception("Could not send websocket payload")
            return False

    def _schedule_payload(self, payload: bytes) -> bool:
        with self._state_lock:
            can_send = self.is_connected and self._proto is not None and not self._closing
        if can_send:
            reactor.callFromThread(self._send_payload, payload)
        return can_send

    @staticmethod
    def _control_payload(identifier: str, topic: str) -> bytes:
        return json.dumps([identifier, {"topicName": topic}]).encode("utf-8")

    def _send_subscription(self, topic: str) -> bool:
        return self._send_payload(
            self._control_payload(WebsocketV1Transport.MSG_SUB, topic)
        )

    def create_socket_subscription(self, msg_type: str, topic: str, callback) -> None:
        """Register a desired subscription and send it if the socket is open."""
        del msg_type  # The server determines the type from its advertised topic list.
        with self._state_lock:
            if self._closing:
                raise RosboardConnectionError("Cannot subscribe a closed client")
            self.socket_subscriptions[topic] = callback
        self.logger.info("Registering subscriber for topic %s", topic)
        self._schedule_payload(
            self._control_payload(WebsocketV1Transport.MSG_SUB, topic)
        )

    def refresh_socket_subscription(self, topic: str) -> bool:
        """Re-send one subscription without removing its local callback."""
        with self._state_lock:
            registered = topic in self.socket_subscriptions
            connected = self.is_connected and self._proto is not None
        if not registered or not connected:
            return False

        def refresh() -> None:
            self._send_payload(
                self._control_payload(WebsocketV1Transport.MSG_UNSUB, topic)
            )
            self._send_payload(
                self._control_payload(WebsocketV1Transport.MSG_SUB, topic)
            )

        reactor.callFromThread(refresh)
        return True

    def destroy_socket_subscription(self, topic_name: str) -> bool:
        with self._state_lock:
            existed = self.socket_subscriptions.pop(topic_name, None) is not None
        if existed:
            self.logger.info("Destroying subscriber for topic %s", topic_name)
            self._schedule_payload(
                self._control_payload(WebsocketV1Transport.MSG_UNSUB, topic_name)
            )
        return existed

    def destroy_socket_publisher(self, topic_name: str) -> bool:
        self.logger.info("Destroying remote publisher for topic %s", topic_name)
        return self._schedule_payload(
            self._control_payload(WebsocketV1Transport.MSG_UNPUB, topic_name)
        )

    def send_ros_message(self, ros_message_dict: dict) -> bool:
        """Send a local ROS message, or drop it while disconnected.

        Commands are intentionally not queued during reconnect: replaying an old
        velocity or arm command after the link returns would be unsafe.
        """
        try:
            payload = json.dumps(
                [WebsocketV1Transport.MSG_MSG, ros_message_dict]
            ).encode("utf-8")
        except (TypeError, ValueError):
            self.logger.exception("Could not serialize outgoing ROS message")
            return False
        return self._schedule_payload(payload)

    def set_available_topics(self, topics: dict) -> None:
        with self._state_lock:
            self.available_topics = dict(topics)
            self._topics_event.set()

    def get_available_topics(self) -> list:
        with self._state_lock:
            return list(self.available_topics)

    def is_topic_available(self, topic: str) -> bool:
        with self._state_lock:
            return topic in self.available_topics

    def get_topic_type(self, topic: str) -> str:
        with self._state_lock:
            try:
                return self.available_topics[topic]
            except KeyError:
                raise RosboardConnectionError(
                    "Topic is not available: {}".format(topic)
                )

    def force_reconnect(self, reason: str = "stale data") -> bool:
        """Close the current socket and let the factory establish a fresh one."""
        now = time.monotonic()
        cooldown = self._env_float("WS_FORCE_RECONNECT_COOLDOWN_S", 10.0)
        with self._state_lock:
            proto = self._proto
            if (
                self._closing
                or proto is None
                or now - self._last_forced_reconnect < cooldown
            ):
                return False
            self._last_forced_reconnect = now
        self.logger.warning("Forcing websocket reconnect: %s", reason)

        def close_current() -> None:
            try:
                proto.sendClose(code=1000, reason=reason.encode("utf-8")[:120])
            except Exception:
                try:
                    proto.transport.abortConnection()
                except Exception:
                    self.logger.exception("Could not force websocket reconnect")

        reactor.callFromThread(close_current)
        return True

    def get_health(self) -> dict:
        now = time.monotonic()
        with self._state_lock:
            return {
                "connected": self.is_connected,
                "closing": self._closing,
                "generation": self.connection_generation,
                "connection_age_s": self._age(now, self._open_time),
                "last_rx_age_s": self._age(now, self._last_rx_time),
                "last_message_age_s": self._age(now, self._last_message_time),
                "subscriptions": len(self.socket_subscriptions),
                "dropped_messages": self._dispatcher.dropped_messages,
                "last_error": self._last_connection_error,
            }

    def _start_health_logging(self) -> None:
        interval = self._env_float("WS_HEALTH_LOG_INTERVAL_S", 60.0)
        if interval <= 0:
            return
        if self._health_log is None:
            self._health_log = LoopingCall(self._log_health)
        if not self._health_log.running:
            self._health_log.start(interval, now=False)

    def _log_health(self) -> None:
        health = self.get_health()
        self.logger.info(
            "Connection health: connected=%s generation=%s "
            "rx_age=%s data_age=%s subscriptions=%s dropped=%s",
            health["connected"],
            health["generation"],
            health["last_rx_age_s"],
            health["last_message_age_s"],
            health["subscriptions"],
            health["dropped_messages"],
        )

    def clientConnectionLost(self, connector, reason) -> None:
        with self._state_lock:
            closing = self._closing
            self.is_connected = False
            self._connected_event.clear()
            if self._last_connection_error is None:
                self._last_connection_error = str(reason)
        if closing:
            self._closed_event.set()
            return
        self.logger.warning("Lost connection with %s: %s", self.url, reason)
        ReconnectingClientFactory.clientConnectionLost(self, connector, reason)

    def clientConnectionFailed(self, connector, reason) -> None:
        with self._state_lock:
            closing = self._closing
            self.is_connected = False
            self._connected_event.clear()
            self._last_connection_error = str(reason)
        if closing:
            self._closed_event.set()
            return
        self.logger.warning("Connection attempt to %s failed: %s", self.url, reason)
        ReconnectingClientFactory.clientConnectionFailed(self, connector, reason)

    def close(self, timeout: float = 2.0) -> None:
        """Stop reconnects, callbacks and the physical connection.

        The shared reactor deliberately remains alive so another robot can be
        connected in the same application process.
        """
        with self._state_lock:
            if self._closing:
                already_closing = True
            else:
                already_closing = False
                self._closing = True
                self.socket_subscriptions.clear()
                self.available_topics = {}
                self._topics_event.clear()
                self._connected_event.clear()
                proto = self._proto
                connector = self.connector
        if already_closing:
            self._closed_event.wait(timeout)
            return

        self._dispatcher.stop()

        def shutdown_connection() -> None:
            try:
                ReconnectingClientFactory.stopTrying(self)
            except Exception:
                self.logger.debug("No pending reconnect to cancel", exc_info=True)
            try:
                if self._health_log is not None and self._health_log.running:
                    self._health_log.stop()
            except Exception:
                self.logger.debug("Could not stop health logger", exc_info=True)
            try:
                if proto is not None:
                    proto.sendClose(code=1000, reason=b"client closed")
                elif connector is not None:
                    connector.disconnect()
                else:
                    self._closed_event.set()
            except Exception:
                self.logger.debug("Graceful websocket close failed", exc_info=True)
                try:
                    if connector is not None:
                        connector.disconnect()
                finally:
                    self._closed_event.set()

        if reactor.running:
            reactor.callFromThread(shutdown_connection)
            closed = self._closed_event.wait(timeout)
            if not closed:
                # A peer may ignore the close handshake.  Deterministic
                # disconnect is more important than waiting indefinitely,
                # especially before connecting a different robot.
                def abort_connection() -> None:
                    try:
                        if proto is not None and proto.transport is not None:
                            proto.transport.abortConnection()
                        elif connector is not None:
                            connector.disconnect()
                    except Exception:
                        self.logger.debug(
                            "Forced websocket abort failed", exc_info=True
                        )
                    finally:
                        self._closed_event.set()

                reactor.callFromThread(abort_connection)
                self._closed_event.wait(0.5)
        else:
            self._closed_event.set()

        with self._state_lock:
            self._proto = None
            self.is_connected = False
        self.logger.info("Rosboard client closed")

    def stop_reactor(self) -> None:
        """Compatibility shim: close this client, not the shared reactor."""
        self.logger.warning(
            "stop_reactor() is deprecated; closing only this rosboard client"
        )
        self.close()

    @classmethod
    def shutdown_reactor(cls) -> None:
        """Stop Twisted during final process shutdown."""
        _ReactorRunner.stop_for_process_exit()
