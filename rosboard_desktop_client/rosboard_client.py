#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Code Information:
    Maintainer: Eng. Pedro Alejandro Gonzalez B
	Mail: pedro@kiwibot.com
"""

# =============================================================================
import os
import time
import yaml
import json
import rclpy
import threading
from rclpy.node import Node
import logging

from twisted.internet import reactor
from python_utils.profilers import profile

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


class WebsocketV1Transport:
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
    jpeg_rgb_decode = lambda binary_data: simplejpeg.decode_jpeg(
        base64.b64decode(binary_data), colorspace="bgr"
    )
    default_decode = lambda binary_data: base64.b64decode(binary_data)
    decoders = {
        "sensor_msgs/msg/Image": {"_data_jpeg": jpeg_rgb_decode},
        "sensor_msgs/msg/CompressedImage": {"_data_jpeg": jpeg_rgb_decode},
        "nav_msgs/msg/OccupancyGrid": {"_data_jpeg": jpeg_rgb_decode},
        "sensor_msgs/msg/PointCloud2": {"_data_uint16.points": default_decode},
        "sensor_msgs/msg/LaserScan": {
            "_ranges_uint16.points": default_decode,
            "_intensities_uint16.points": default_decode,
        },
    }

    def decode_field(self, dictionary, key, decode_function):
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
    def decode_binary_fields(self, rosboard_data):
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
            raise ModuleNotFoundError(f"Could not import {topic_class_name}")
        self.publisher = parent_node.create_publisher(
            msg_type=self.topic_class, topic=topic_name, qos_profile=1
        )
        self.logger = parent_node.get_logger()

    def publish(self, msg: any):
        if not isinstance(msg, self.topic_class):
            self.logger.error(
                f"tried to publish a message of type {type(msg).__name__} while a publisher was created for type {self.topic_class_name}"
            )
            return
        if self.publisher is None:
            self.logger.error(f"tried to publish message but no node was provided")
            return
        self.publisher.publish(msg)

    def parse_and_publish(self, rosboard_data):
        self.publish(self.parse_message(rosboard_data))

    def parse_message(self, rosboard_data):
        rosboard_dict = rosboard_data[1]
        message = convert_dictionary_to_ros_message(
            self.topic_class_name.replace(".", "/"), rosboard_data[1], strict_mode=False
        )
        return message
        # self.publisher.publish(msg)


class ImagePublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args) -> None:
        super().__init__(parent_node, topic_name, "sensor_msgs.msg.Image")
        self.bridge = CvBridge()

    def parse_message(self, rosboard_data):
        # print(data)
        image_bytes = rosboard_data[1]["_data_jpeg"]

        base_image = convert_dictionary_to_ros_message(
            "sensor_msgs/msg/Image", rosboard_data[1], strict_mode=False
        )
        # image.encoding = "bgr8"
        image = self.bridge.cv2_to_imgmsg(image_bytes, encoding="passthrough")
        image.header = base_image.header
        return image

    @classmethod
    def supported_msg_types(self):
        return ["sensor_msgs/msg/Image"]


class PointCloudPublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args) -> None:
        super().__init__(parent_node, topic_name, "sensor_msgs.msg.PointCloud2")

    def parse_message(self, rosboard_data):
        binary_data = rosboard_data[1]["_data_uint16"]["points"]
        xmin, xmax, ymin, ymax, zmin, zmax = rosboard_data[1]["_data_uint16"]["bounds"]
        points_array = np.frombuffer(binary_data, np.uint16)
        points_array = points_array.reshape(points_array.shape[0] // 3, 3).astype(
            np.float32
        )
        points_array[:, 0] = self.scale_back_array(points_array[:, 0], xmin, xmax)
        points_array[:, 1] = self.scale_back_array(points_array[:, 1], ymin, ymax)
        points_array[:, 2] = self.scale_back_array(points_array[:, 2], zmin, zmax)
        base_pc_msg = convert_dictionary_to_ros_message(
            "sensor_msgs/msg/PointCloud2", rosboard_data[1], strict_mode=False
        )
        # xyz_pc = point_cloud2.create_cloud_xyz32(header=base_pc_msg.header, points=points_array) # -> avoided for efficiency reasons
        xyz_pc = point_cloud2.create_cloud_xyz32(header=base_pc_msg.header, points=[])
        xyz_pc.width = points_array.shape[0]
        xyz_pc.point_step = 12
        xyz_pc.row_step = xyz_pc.width * xyz_pc.point_step
        xyz_pc.data = points_array.tobytes()
        return xyz_pc

    def scale_back_array(self, array: np.ndarray, min_val: float, max_val: float):
        return (array / 65535.0) * (max_val - min_val) + min_val

    @classmethod
    def supported_msg_types(self):
        return ["sensor_msgs/msg/PointCloud2"]


class PublisherManager:
    available_publishers = [ImagePublisher, PointCloudPublisher]
    default_publisher = GenericPublisher

    @classmethod
    def getDefaultPublisherForType(self, topic_type: str):
        supported_publishers = list(
            filter(
                lambda publisher: topic_type in publisher.supported_msg_types(),
                self.available_publishers,
            )
        )
        if len(supported_publishers):
            return supported_publishers[0]
        return self.default_publisher


class RosboardClientProtocol(WebSocketClientProtocol):
    def onConnect(self, response):
        print(response)
        self.factory.logger.info(f"Server connected: {response.peer}")

    def onOpen(self):
        self.factory.logger.info(f"Communication opened")
        self.factory.ready(self)

    def onClose(self, wasClean, code, reason):
        self.factory.logger.warning(
            f"Communication closed. reason: {reason} was clean: {wasClean}, code: {code}"
        )

    def onMessage(self, payload, isBinary):
        if not isBinary:
            data = json.loads(payload.decode("utf8"))

        # Only process messages for now
        if data[0] == WebsocketV1Transport.MSG_MSG:
            data = RosboardDecoder.decode_binary_fields(data)
            self.factory.socket_subscriptions[data[1]["_topic_name"]](data)
            # print(f"got message on topic {data[1]}")
        if data[0] == WebsocketV1Transport.MSG_TOPICS:
            self.factory.set_available_topics(data[1])

    def send_message(self, payload):
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
        self.logger = logging.getLogger("rosboard_client")

        self.socket_subscriptions = {}
        self.available_topics = {}

        # Create socket connection
        socket_url = "ws://" + host + "/rosboard/v1"
        WebSocketClientFactory.__init__(self, url=socket_url)
        self.logger.info(f"connecting to {socket_url}")
        self.connector = connectWS(self)
        self._proto = None
        self._thread = threading.Thread(target=reactor.run, args=(False,))
        self._thread.daemon = True
        self._thread.start()

        # Connection timeout if taking too long
        connection_request_time = time.time()
        while self._proto is None:
            time.sleep(0.05)
            if time.time() - connection_request_time > connection_timeout:
                self.logger.error(f"Connection attempt to {socket_url} timed out")
                raise Exception("Connection timed out")

        # Check if rosboard returns the topics available on the robot
        while not self.available_topics:
            time.sleep(0.05)
            if time.time() - connection_request_time > connection_timeout:
                self.logger.error(
                    f"{socket_url} did not send the available topics. Timed out"
                )
                raise Exception("Available topics not received")

        self.logger.info("available topics advertised by server")

    def create_socket_subscription(self, msg_type, topic, callback):
        # topic_type = ""
        # if not self.is_topic_available(topic):
        #     if not "type" in kwargs:
        #         self.logger.warning(
        #             f"Will not to topic that is not available: {topic}, unable to determine type",
        #         )
        #         return
        #     else:
        #         topic_type = kwargs["type"]
        # else:
        #     topic_type = self.get_topic_type(topic)

        # # in case python like message types are used
        # topic_type.replace(".", "/")
        # publisher = PublisherManager.getDefaultPublisherForType(topic_type)
        self.logger.info(f"creating subscriber for topic {topic}")
        self.socket_subscriptions[topic] = callback
        self._proto.send_message(
            json.dumps([WebsocketV1Transport.MSG_SUB, {"topicName": topic}]).encode(
                "utf-8"
            ),
        )

    def destroy_socket_subscription(self, topic_name):
        raise NotImplementedError

    def ready(self, proto):
        self._proto = proto

    def set_available_topics(self, topics: dict):
        self.available_topics = topics

    def is_topic_available(self, topic: str):
        return topic in self.available_topics.keys()

    def get_topic_type(self, topic: str):
        if self.is_topic_available(topic):
            return self.available_topics[topic]
        else:
            self.logger.error(
                f"Unable request type of topic that is not available: {topic}"
            )
            raise Exception("Cannot request type of topic that is not available")

    def clientConnectionLost(self, connector, reason):
        self.logger.error(f"Lost connection with {self.url}, reason: {reason}")
        raise Exception("Connection Lost")

    def clientConnectionFailed(self, connector, reason):
        self.logger.error(f"Failed to connect to {self.url}, reason: {reason}")
        raise Exception("Unable to connect")


class RosboardYamlNode(Node):
    def __init__(self):
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
    Main Functions of Local Console Node
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
