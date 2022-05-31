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


class RosboardClientProtocol(WebSocketClientProtocol):
    def onConnect(self, response):
        print(response)
        self.factory.logger.info(f"Server connected: {response.peer}")

    def onOpen(self):
        self.factory.logger.info(f"Communication opened")

    def onClose(self, wasClean, code, reason):
        self.factory.logger.warning(
            f"Communication closed. reason: {reason} was clean: {wasClean}, code: {code}"
        )

    def onMessage(self, payload, isBinary):
        if not isBinary:
            data = json.loads(payload.decode("utf8"))
            print("Text message received: {}".format(payload.decode("utf8")))

            # image_bytes = simplejpeg.decode_jpeg(
            # base64.b64decode(data[1]["_data_jpeg"]), colorspace="bgr"
            # )
            # # print(image_bytes.shape)
            # cv2.imshow("troll", image_bytes)
            # image = cv2.Mat(image_bytes)
            # cv2.waitKey(1)
            # print(
            #     "image_received: av freq",
            #     self.measurements / (time.time() - self.last_time),
            #     "dt",
            #     # time.time() - self.last_time,
            # )
            # self.last_time = time.time()
        # reactor.callLater(1, self.sendHello)


class RosboardClient(WebSocketClientFactory, Node):
    protocol = RosboardClientProtocol

    def __init__(self):
        Node.__init__(self, node_name="rosboard_client")
        # self.protocol.set_parent_node(parent_node=parent_node)
        # self.setProtocolOptions()
        self.logger = self.get_logger()
        path = "/rosboard/v1"
        host = "127.0.0.1:8888"
        socket_url = "ws://" + host + path
        WebSocketClientFactory.__init__(self, url=socket_url)
        print(socket_url)
        connector = connectWS(self)

        self._thread = threading.Thread(target=reactor.run, args=(False,))
        self._thread.daemon = True
        self._thread.start()

    def subscribe(self, topic_name):
        self.protocol.sendMessage(
            json.dumps(
                [WebsocketV1Transport.MSG_SUB, {"topicName": "/camera/color/image_raw"}]
            ).encode("utf-8")
        )

    def unsubscribe(self, topic_name):
        pass

    def clientConnectionLost(self, connector, reason):
        self.logger.error(f"Lost connection with {self.url}, reason: {reason}")
        reactor.stop()
        exit(1)

    def clientConnectionFailed(self, connector, reason):
        self.logger.error(f"Failed to connect to {self.url}, reason: {reason}")
        reactor.stop()
        exit(1)


# =============================================================================
def main(args=None):
    """!
    Main Functions of Local Console Node
    """
    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the
    # executor is shutdown.
    rosboard_client = RosboardClient()

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
