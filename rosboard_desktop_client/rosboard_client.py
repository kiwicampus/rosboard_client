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
from ament_index_python.packages import get_package_share_directory
import threading
from rclpy.node import Node

from twisted.internet import reactor
from twisted.python import log

from rclpy.executors import MultiThreadedExecutor

from autobahn.twisted.websocket import (
    WebSocketClientFactory,
    WebSocketClientProtocol,
    connectWS,
)

# Image dependencies
from sensor_msgs.msg import Image
import base64
import cv2
import simplejpeg


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


class ImagePublisher:
    def __init__(self, parent_node: Node, topic_name: str) -> None:
        self.parent_node = parent_node
        self.publisher = parent_node.create_publisher(Image, topic_name, 1)

    def publish(self, data: dict):
        image_bytes = simplejpeg.decode_jpeg(
            base64.b64decode(data[1]["_data_jpeg"]), colorspace="bgr"
        )
        # print(image_bytes.shape)
        cv2.imshow("troll", image_bytes)
        cv2.waitKey(1)
        self.publisher.publish(Image())


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
        if data[0] != WebsocketV1Transport.MSG_MSG:
            print("Text message received: {}".format(payload.decode("utf8")))
            return

        print(f"got message on topic {data[1]['_topic_name']}")
        self.factory.socket_subscriptions[data[1]["_topic_name"]].publish(data)

    def send_message(self, payload):
        return reactor.callFromThread(
            self.sendMessage,
            payload,
            isBinary=False,
            fragmentSize=None,
            sync=False,
            doNotCompress=False,
        )


class RosboardClient(WebSocketClientFactory, Node):
    protocol = RosboardClientProtocol

    def __init__(self, host: str, parent_node: Node):
        self.parent_node = parent_node
        self.logger = parent_node.get_logger()

        self.socket_subscriptions = {}

        # Create socket connection
        socket_url = "ws://" + host + "/rosboard/v1"
        WebSocketClientFactory.__init__(self, url=socket_url)
        self.logger.info(f"connecting to {socket_url}")
        self.connector = connectWS(self)
        self._proto = None
        self._thread = threading.Thread(target=reactor.run, args=(False,))
        self._thread.daemon = True
        self._thread.start()

    def create_socket_subscription(self, topic_name: str):
        self.socket_subscriptions[topic_name] = ImagePublisher(
            self.parent_node, topic_name=topic_name
        )
        self._proto.send_message(
            json.dumps(
                [WebsocketV1Transport.MSG_SUB, {"topicName": topic_name}]
            ).encode("utf-8"),
        )

    def destroy_socket_subscription(self, topic_name):
        pass

    def ready(self, proto):
        self._proto = proto

    def clientConnectionLost(self, connector, reason):
        self.logger.error(f"Lost connection with {self.url}, reason: {reason}")
        reactor.stop()
        exit(1)

    def clientConnectionFailed(self, connector, reason):
        self.logger.error(f"Failed to connect to {self.url}, reason: {reason}")
        reactor.stop()
        exit(1)


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

        self.client = RosboardClient(host=host, parent_node=self)

        time.sleep(2)
        # Subscribe to rosboard topics
        for topic in topics_to_subscribe:
            self.client.create_socket_subscription(topic)


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
