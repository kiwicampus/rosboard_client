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
from math import nan
import os
import time
import yaml
import json
import rclpy
import threading
from rclpy.node import Node
import logging

from twisted.internet import reactor

# from python_utils.profilers import profile

from rclpy.executors import MultiThreadedExecutor

from autobahn.twisted.websocket import (
    WebSocketClientFactory,
    WebSocketClientProtocol,
    connectWS,
)

import sensor_msgs

# Image dependencies
from sensor_msgs.msg import Image
import base64
import cv2
import simplejpeg
from cv_bridge import CvBridge
from rclpy_message_converter.message_converter import convert_dictionary_to_ros_message


# PointCloud dependencies
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np


# Occupancy Grid dependencies
import nav_msgs.msg


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

    # Function to decode jpeg encoded images
    jpeg_bgr_decode = lambda binary_data: simplejpeg.decode_jpeg(
        base64.b64decode(binary_data), colorspace="bgr"
    )
    # Function to decode png encoded, single channel images
    png_gray_decode = lambda binary_data: cv2.imdecode(
        np.fromstring(base64.b64decode(binary_data), dtype="uint8"),
        cv2.IMREAD_UNCHANGED,
    )
    # Function to decode b64 encoded data
    default_decode = lambda binary_data: base64.b64decode(binary_data)

    # dictionary storing type of decoding to apply to each field
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


class GenericPublisher:
    def __init__(
        self,
        parent_node: Node,
        topic_name: str,
        topic_class_name: str,
    ) -> None:
        """! Class to transform rosboard data into ROS messages and re publish them in
        the local system when received. It should work with all messages types whenever
        they can be imported. It makes uses of the rclpy message converter library
        @param parent_node (Node) a Node object to create publishers
        @param topic_name (str) The name of the topic to republish the messages
        @param topic_class_name (str) The type of the messages.
        Raises:
            ModuleNotFoundError: whenever a message type cannot be imported
        """
        self.topic_class_name = topic_class_name
        self.parent_node = parent_node
        if parent_node is None:
            self.publisher = None
            self.logger = logging.getLogger("rosboard_client")
            self.logger.warning(
                f"No parent node was provided. Will not be able to publish messages on topic {topic_name}"
            )
            return
        try:
            self.topic_class = eval(topic_class_name.replace("/", "."))
        except ModuleNotFoundError:
            raise ModuleNotFoundError(
                f"Could not import {topic_class_name}. Is your workspace sourced?"
            )
        self.publisher = parent_node.create_publisher(
            msg_type=self.topic_class, topic=topic_name, qos_profile=1
        )
        self.logger = parent_node.get_logger()

    def publish(self, msg: any) -> None:
        """!
        Function to publish ros messages on the specified topic
        @param msg (any) The message to publish
        """
        if not isinstance(msg, self.topic_class):
            self.logger.error(
                f"tried to publish a message of type {type(msg).__name__} while a publisher was created for type {self.topic_class_name}"
            )
            return
        if self.publisher is None:
            self.logger.error(f"tried to publish message but no node was provided")
            return
        self.publisher.publish(msg)

    def parse_and_publish(self, rosboard_data: list) -> None:
        """!
        Function to parse a message from rosboard data and publish it
        @param rosboard_data (list) the data from rosboard. the binary fields must have already been decoded
        """
        self.publish(self.parse_message(rosboard_data))

    def parse_message(self, rosboard_data: list) -> list:
        """!
        Function to convert the rosboard data to a ROS message. If a field in the message
        is not contained in the rosboard data it will be left empty on the ROS message
        @param rosboard_data (list) The rosboard data, the binary fields must have already been decoded
        @return any The ROS message
        """
        # Make use of rclpy message converter. with strict_mode=False fields not present
        # in the dictionary will be left empty on the ROS message
        message = convert_dictionary_to_ros_message(
            self.topic_class_name.replace(".", "/"), rosboard_data[1], strict_mode=False
        )
        return message


class ImagePublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args) -> None:
        """!
        Class to parse ROS images from rosboard image data. Inherits from the GenericPublisher
        Refer to https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py
        to see how data is encoded and compressed
        @param parent_node (Node) A node object to create the image publisher
        @param topic_name (str) The name of the image topic to republish messages
        """
        super().__init__(parent_node, topic_name, "sensor_msgs.msg.Image")
        self.bridge = CvBridge()

    def parse_message(self, rosboard_data: list) -> sensor_msgs.msg.Image:
        """!
        Overrides the parse_message function from the GenericPublisher to get a ROS
        image from the jpeg data
        @param rosboard_data (list) The rosboard data, the binary fields must have already been decoded
        @return sensor_msgs.msg.Image The ROS Image message
        """
        image_bytes = rosboard_data[1]["_data_jpeg"]
        base_image = convert_dictionary_to_ros_message(
            "sensor_msgs/msg/Image", rosboard_data[1], strict_mode=False
        )
        image = self.bridge.cv2_to_imgmsg(image_bytes, encoding="passthrough")
        image.header = base_image.header
        return image

    @classmethod
    def supported_msg_types(self) -> list:
        """!
        Get the message types supported by the ImagePublisher
        @return list a list with all the supported message types
        """
        return ["sensor_msgs/msg/Image"]


class OccupancyGridPublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args) -> None:
        """!
        Class to parse ROS OccupancyGrids from rosboard data. Inherits from the GenericPublisher
        This only works with our fork, because encoding for occupancy grids was changed to PNG
        Refer to https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py
        to see how data is encoded and compressed
        @param parent_node (Node) A node object to create the OccupancyGrid publisher
        @param topic_name (str) The name of the OccupancyGrid topic to republish messages
        """
        super().__init__(parent_node, topic_name, "nav_msgs.msg.OccupancyGrid")

    def parse_message(self, rosboard_data) -> nav_msgs.msg.OccupancyGrid:
        """!
        Overrides the parse_message function from the GenericPublisher to get a ROS
        image from the png data
        @param rosboard_data (list) The rosboard data, the binary fields must have already been decoded
        @return sensor_msgs.msg.Image The ROS Image message
        """
        image_bytes = rosboard_data[1]["_data_jpeg"]
        base_occupancy_grid = convert_dictionary_to_ros_message(
            "nav_msgs/msg/OccupancyGrid", rosboard_data[1], strict_mode=False
        )
        # adjust resolution for rosboard subsampling
        base_occupancy_grid.info.resolution = base_occupancy_grid.info.resolution * (
            base_occupancy_grid.info.width / image_bytes.shape[0]
        )
        # adjust size for rosboard subsampling
        base_occupancy_grid.info.width = image_bytes.shape[0]
        base_occupancy_grid.info.height = image_bytes.shape[1]
        occupancy_grid_array = image_bytes.astype(np.int8)

        base_occupancy_grid.data = (
            occupancy_grid_array.flatten().astype(np.int8).tolist()
        )
        return base_occupancy_grid

    @classmethod
    def supported_msg_types(self) -> list:
        """!
        Get the message types supported by the OccupancyGridPublisher
        @return list a list with all the supported message types
        """
        return ["nav_msgs/msg/OccupancyGrid"]


class PointCloudPublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args) -> None:
        """!
        Class to parse ROS PointCloud2 from rosboard pointcloud data. Inherits from the GenericPublisher
        Keep in mind that rosboard only sends xyz data, removing all the other PointFields
        Refer to https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py
        to see how data is encoded and compressed
        @param parent_node (Node) A node object to create the PointCloud2 publisher
        @param topic_name (str) The name of the PointCloud2 topic to republish messages
        """
        super().__init__(parent_node, topic_name, "sensor_msgs.msg.PointCloud2")

    def parse_message(self, rosboard_data: list) -> sensor_msgs.msg.PointCloud2:
        """!
        Overrides the parse_message function from the GenericPublisher to get a ROS
        PointCloud from the rosboard decoded data
        @param rosboard_data (list) The rosboard data, the binary fields must have already been decoded
        @return sensor_msgs.msg.PointCloud2 The ROS PointCloud2 message
        """
        binary_data = rosboard_data[1]["_data_uint16"]["points"]
        xmin, xmax, ymin, ymax, zmin, zmax = rosboard_data[1]["_data_uint16"]["bounds"]
        points_array = np.frombuffer(binary_data, np.uint16)
        # Points are contained in the rosboard array sequentially as x-y-z, x-y-z ...
        # Thus a 3 channel np array is created where each channel contains the points
        # in each coordinate.
        points_array = points_array.reshape(points_array.shape[0] // 3, 3).astype(
            np.float32
        )
        # Rosboard sends points scaled between 0 and 65535 where 0 maps to xmin and 65535 maps to xmax
        points_array[:, 0] = self.scale_back_array(points_array[:, 0], xmin, xmax)
        points_array[:, 1] = self.scale_back_array(points_array[:, 1], ymin, ymax)
        points_array[:, 2] = self.scale_back_array(points_array[:, 2], zmin, zmax)
        base_pc_msg = convert_dictionary_to_ros_message(
            "sensor_msgs/msg/PointCloud2", rosboard_data[1], strict_mode=False
        )
        # avoided for efficiency reasons
        # xyz_pc = point_cloud2.create_cloud_xyz32(header=base_pc_msg.header, points=points_array)
        xyz_pc = point_cloud2.create_cloud_xyz32(header=base_pc_msg.header, points=[])
        xyz_pc.width = points_array.shape[0]
        xyz_pc.point_step = 12
        xyz_pc.row_step = xyz_pc.width * xyz_pc.point_step
        # Since the array is already made by floats, it can be directly turned to bytes and copied
        # to the message data field
        xyz_pc.data = points_array.tobytes()
        return xyz_pc

    def scale_back_array(
        self, array: np.ndarray, min_val: float, max_val: float
    ) -> np.ndarray:
        """!
        Scale array back to meters according to the rosboard. Rosboard sends
        points scaled between 0 and 65535 where 0 maps to xmin and 65535 maps to xmax
        @param array (np.ndarray) the array containing the scaled data
        @param min_val (float) the min value in meters
        @param max_val (float) the max value in meters
        @return np.ndarray the array with the values in meters
        """
        return (array / 65535.0) * (max_val - min_val) + min_val

    @classmethod
    def supported_msg_types(self) -> list:
        """!
        Get the message types supported by the PointCloudPublisher
        @return list a list with all the supported message types
        """
        return ["sensor_msgs/msg/PointCloud2"]


class LaserScanPublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args) -> None:
        """!
        Class to parse ROS LaserScans from rosboard laserscan data. Inherits from the GenericPublisher
        Refer to https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py
        to see how data is encoded and compressed
        @param parent_node (Node) A node object to create the PointCloud2 publisher
        @param topic_name (str) The name of the PointCloud2 topic to republish messages
        """
        super().__init__(parent_node, topic_name, "sensor_msgs.msg.LaserScan")

    def parse_message(self, rosboard_data: list) -> sensor_msgs.msg.LaserScan:
        """!
        Overrides the parse_message function from the GenericPublisher to get a ROS
        LaserScan from the rosboard decoded data
        @param rosboard_data (list) The rosboard data, the binary fields must have already been decoded
        @return sensor_msgs.msg.LaserScan The ROS LaserScan message
        """
        binary_ranges = rosboard_data[1]["_ranges_uint16"]["points"]
        binary_intensities = rosboard_data[1]["_intensities_uint16"]["points"]
        rmin, rmax = rosboard_data[1]["_ranges_uint16"]["bounds"]
        imin, imax = rosboard_data[1]["_intensities_uint16"]["bounds"]
        # https://stackoverflow.com/questions/53971620/cant-modify-numpy-array
        ranges_array = np.frombuffer(binary_ranges, np.uint16).astype(np.float32).copy()
        # zero or nan points are encoded by rosboard with 65535
        # see: https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py#L351
        invalid_idxs = ranges_array == 65535.0
        intensities_array = (
            np.frombuffer(binary_intensities, np.uint16).astype(np.float32).copy()
        )
        # Rosboard sends points scaled between 0 and 65534 where 0 maps to xmin and 65534 maps to xmax
        # 65535 is invalid value (nan/-inf/inf)
        ranges_array = self.scale_back_array(ranges_array, rmin, rmax)
        intensities_array = self.scale_back_array(intensities_array, imin, imax)
        ranges_array[invalid_idxs] = 0.0
        base_laser_msg = convert_dictionary_to_ros_message(
            "sensor_msgs/msg/LaserScan", rosboard_data[1], strict_mode=False
        )
        base_laser_msg.ranges = ranges_array.flatten().tolist()
        base_laser_msg.intensities = intensities_array.flatten().tolist()
        return base_laser_msg

    def scale_back_array(
        self, array: np.ndarray, min_val: float, max_val: float
    ) -> np.ndarray:
        """!
        Scale array back to meters according to the rosboard. Rosboard sends
        points scaled between 0 and 65534 where 0 maps to xmin and 65535 maps to xmax
        65535 is invalid value (nan/-inf/inf)
        @param array (np.ndarray) the array containing the scaled data
        @param min_val (float) the min value in meters
        @param max_val (float) the max value in meters
        @return np.ndarray the array with the values in meters
        """
        return (array / 65534.0) * (max_val - min_val) + min_val

    @classmethod
    def supported_msg_types(self):
        """!
        Get the message types supported by the LaserScanPublishers
        @return list a list with all the supported message types
        """
        return ["sensor_msgs/msg/LaserScan"]


class PublisherManager:
    """!
    Class to manage available publishers and return the appropriate
    publisher for each message type
    """

    available_publishers = [
        ImagePublisher,
        PointCloudPublisher,
        OccupancyGridPublisher,
        LaserScanPublisher,
    ]
    default_publisher = GenericPublisher

    @classmethod
    def getDefaultPublisherForType(self, topic_type: str) -> GenericPublisher:
        """! Function to get a publisher class for a given message type
        @param topic_type (str) the message type using the rosboard formar. Ex: nav_msgs/msg/Path
        @return Publisher class with the most appropriate publisher for the given topic type
        """
        supported_publishers = list(
            filter(
                lambda publisher: topic_type in publisher.supported_msg_types(),
                self.available_publishers,
            )
        )
        # In case no special publishers are found for the topic type, use a generic publisher
        if len(supported_publishers):
            return supported_publishers[0]
        return self.default_publisher


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
            self.factory.socket_subscriptions[data[1]["_topic_name"]](data)
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


class RosboardClient(WebSocketClientFactory):
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
        socket_url = "ws://" + host + "/rosboard/v1"
        WebSocketClientFactory.__init__(self, url=socket_url)
        self.logger.info(f"connecting to {socket_url}")
        self.connector = connectWS(self)
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
        self.logger.info(f"Destroying subscriber for topic {topic}")
        if topic in self.socket_subscriptions.keys():
            # remove the subscription from the dictionary
            self.socket_subscriptions.pop(topic)
        else:
            self.logger.warning(
                f"No subscription had been registered for topic {topic_name}"
            )
        self._proto.send_message(
            # rosboard expects a message like this" ["u", {topicName: xxx}] to destroy the subscription
            json.dumps([WebsocketV1Transport.MSG_UNSUB, {"topicName": topic}]).encode(
                "utf-8"
            ),
        )

    def ready(self, proto: WebSocketClientFactory) -> None:
        """!
        Function to set the protocol object in the client. This allows to send messages
        to the server
        @param proto (WebSocketClientFactory) the protocol object
        """
        self._proto = proto

    def set_available_topics(self, topics: dict) -> None:
        """! Function to set the available topics in the client
        @param topics (dict) The dictionary containing all the topics as keys and types as values
        """
        self.available_topics = topics

    def is_topic_available(self, topic: str) -> Bool:
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
        raise Exception("Connection Lost")

    def clientConnectionFailed(self, connector, reason):
        """!
        Function executed when the client cannot establish a connection with the server
        Raises:
            Exception: Always raises an exception. Connection should not fail
        """
        self.logger.error(f"Failed to connect to {self.url}, reason: {reason}")
        raise Exception("Unable to connect")


class RosboardYamlNode(Node):
    def __init__(self):
        """!
        Node object that parses a yaml file to create rosboard subscriptions and
        republishing remote topics on the local machine
        """
        Node.__init__(self, node_name="rosboard_yaml_client")
        self.logger = self.get_logger()

        # Read and parse config file
        config_file_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "topics_to_subscribe.yaml"
        )
        with open(config_file_path, "r") as stream:
            config_dict = yaml.safe_load(stream)
        host = config_dict["url"]
        topics_to_subscribe = config_dict["topics"]

        self.client = RosboardClient(host=host, connection_timeout=5)

        # time.sleep(2)
        # Subscribe to rosboard topics
        for topic in topics_to_subscribe:
            if not self.client.is_topic_available(topic):
                self.logger.warning(
                    f"Will not subscribe to topic that is not available: {topic}, unable to determine type",
                )
                continue

            topic_type = self.client.get_topic_type(topic)
            republisher_class = PublisherManager.getDefaultPublisherForType(topic_type)
            republisher = republisher_class(self, topic, topic_type)
            self.client.create_socket_subscription(
                topic_type, topic, republisher.parse_and_publish
            )


# =============================================================================
def main(args=None):
    """!
    Demonstration of the rosboard client using a yaml file to get topics
    """
    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the
    # executor is shutdown.
    rosboard_client = RosboardYamlNode()

    # Runs callbacks in a pool of threads.
    executor = MultiThreadedExecutor()

    # Execute work and block until the context associated with the
    # executor is shutdown. Callbacks will be executed by the provided
    # executor.
    rclpy.spin(rosboard_client, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rosboard_client.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()

# =============================================================================
