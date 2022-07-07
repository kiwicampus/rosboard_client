#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
"""
Code Information:
    Code Information:
    Maintainer: Eng. Pedro Alejandro Gonzalez B
	Mail: pedro@kiwibot.com
"""

# =============================================================================
import time
import json
import threading
import logging

from twisted.internet import reactor
from twisted.internet.protocol import ReconnectingClientFactory

from autobahn.twisted.websocket import (
    WebSocketClientFactory,
    WebSocketClientProtocol,
    connectWS,
)

# Image dependencies
import base64
import cv2
import simplejpeg

import numpy as np

logging.basicConfig(level=logging.INFO)


def jpeg_bgr_decode(binary_data: str) -> np.ndarray:
    """!
    Function to get a 3 channel np.ndarray image representation from a jpeg
    base64 encoded string
    @param binary_data (str) the base64 encoded jpeg string
    @return np.ndarray the 3 channel RGB image data
    """
    return simplejpeg.decode_jpeg(base64.b64decode(binary_data), colorspace="bgr")


def png_gray_decode(binary_data: str) -> np.ndarray:
    """!
    Function to get a single channel np.ndarray image representation from a png
    base64 encoded string
    @param binary_data (str) the base64 encoded png string
    @return np.ndarray the single channel grayscale image data
    """
    return cv2.imdecode(
        np.fromstring(base64.b64decode(binary_data), dtype="uint8"),
        cv2.IMREAD_UNCHANGED,
    )


def default_decode(binary_data: str) -> bytes:
    """!
    Function to decode a base64 encoded string back to bytes
    @param binary_data (str) the base64 encoded string
    @return bytes the bytes
    """
    return base64.b64decode(binary_data)


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
    PING_SEQ = "s"
    PONG_SEQ = "s"
    PONG_TIME = "t"


class RosboardDecoder:
    """!
    Class to take in rosboard message payloads as dictionaries and decode/decompress
    the binary/encoded fields. The variable `decoders` contains the fields and type
    of decoding for each message.
    """

    decoders = {
        "sensor_msgs/msg/Image": {"_data_jpeg": jpeg_bgr_decode},
        "sensor_msgs/msg/CompressedImage": {"_data_jpeg": jpeg_bgr_decode},
        "nav_msgs/msg/OccupancyGrid": {"_data_jpeg": png_gray_decode},
        "sensor_msgs/msg/PointCloud2": {"_data_uint16.points": default_decode},
        "sensor_msgs/msg/LaserScan": {
            "_ranges_uint16.points": default_decode,
            "_intensities_uint16.points": default_decode,
        },
    }

    def decode_field(self, dictionary: dict, key: str, decode_function) -> dict:
        """!
        Function to decode a binary field on a dictionary by its key and decode function.
        Keys can be passed recursively as `key1.key2` to change fields in a dictionary contained
        inside another.
        @param dictionary (dict) The dictionary containing the field to decode
        @param key (str) The key with the binary data. If the data is contained in a dictionary within
        a dictionary, keys can be passed recursively as `key1.key2`
        @param decode_function (function) The decode function to apply to the binary data on the field
        @return dict The same dictionary with the specified field updated containing the decoded data
        """
        if "." in key:
            new_key = key[key.find(".") + 1 :]
            old_key = key[: key.find(".")]
            dictionary[old_key] = self.decode_field(
                self, dictionary[old_key], new_key, decode_function
            )
        else:
            dictionary[key] = decode_function(dictionary[key])
        return dictionary

    @classmethod
    def decode_binary_fields(self, rosboard_data: list) -> list:
        """!
        Function to decode the binary fields from a rosboard raw message as a list.
        Rosboard messages are lists composed by two elements, the first being a character
        identifying the type of data contained in the message and the second the data itself
        @param rosboard_data (list) the rosboard message after dumping it from json.
        @return list The same rosboard message but with the binary fields containing the decoded
        data
        """
        msg_type = rosboard_data[1]["_topic_type"]
        if not msg_type in self.decoders.keys():
            return rosboard_data

        for field, decoder in self.decoders[msg_type].items():
            rosboard_data[1] = self.decode_field(self, rosboard_data[1], field, decoder)
        return rosboard_data


class RosboardClientProtocol(WebSocketClientProtocol):
    """! Class specifying the rosboard client websocket protocol.
    Inherits from the WebSocketClientProtocol from twisted
    """

    def onConnect(self, response) -> None:
        """!
        Function run when the websocket is successfully connected
        @param response the connection response
        """
        self.factory.logger.info(f"Server connected: {response.peer}")

    def onOpen(self) -> None:
        """!
        Function run when the connection is open. This sets
        the protocol object in the factory using this class
        """
        self.factory.logger.info(f"Communication opened")
        self.factory.ready(self)

    def onClose(self, wasClean, code, reason) -> None:
        """!
        Function run when the connection is closed
        """
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

        self.socket_subscriptions = {}
        self.available_topics = {}

        # Create socket connection
        if host.startswith("wss://") or host.startswith("ws://"):
            socket_url = host + "/rosboard/v1"
        else:
            self.logger.info(
                "websocket protocol not provided in host url. falling back to ws:// as default"
            )
            socket_url = "ws://" + host + "/rosboard/v1"
        WebSocketClientFactory.__init__(self, url=socket_url)
        self.logger.info(f"connecting to {socket_url}")
        self.connector = connectWS(self, connection_timeout)
        # protocol object. Set when the socket is ready
        self._proto = None

        # Run the reactor in a separate thread
        self._thread = threading.Thread(target=reactor.run, args=(False,))
        self._thread.daemon = True
        self._thread.start()

        # Check if connection takes more than the timeout
        connection_request_time = time.time()
        while self._proto is None:
            time.sleep(0.05)
            if time.time() - connection_request_time > connection_timeout:
                self.logger.error(f"Connection attempt to {socket_url} timed out")
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
        Raises:
            Exception: Always raises an exception, the connection with the server should not be lost
        """
        self.logger.error(f"Lost connection with {self.url}, reason: {reason}")
        ReconnectingClientFactory.clientConnectionLost(self, connector, reason)

    def clientConnectionFailed(self, connector, reason):
        """!
        Function executed when the client cannot establish a connection with the server
        Raises:
            Exception: Always raises an exception. Connection should not fail
        """
        self.logger.error(f"Failed to connect to {self.url}, reason: {reason}")
        ReconnectingClientFactory.clientConnectionFailed(self, connector, reason)
