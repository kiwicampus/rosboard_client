#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
"""
License:
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
from threading import Thread
from grpc import Compression
import yaml
import std_msgs.msg, sensor_msgs.msg, nav_msgs.msg, geometry_msgs.msg
import sys
import rclpy
from psutil import net_io_counters
from time import time, sleep
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import os
from rclpy_message_converter import message_converter
import copy

# sys.path.insert(1, "<path_to_roslibpy>/roslibpy/src")
sys.path.insert(1, "/workspace/rover/ros2/src/roslibpy/src")
import roslibpy


def ros2_to_ros1_msg_string(ros2_msg_string: str):
    return ros2_msg_string.replace(".msg.", "/")


class RosBridgeClient(Node):
    def __init__(self) -> None:
        """!
        Class constructor for rosbridge benchmark node
        """
        super().__init__("rosbridge_client_node")
        conf_file_path = yaml.safe_load(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "rosbridge_config.yaml"
            )
        )

        with open(conf_file_path, "r") as stream:
            config_file = yaml.safe_load(stream)

        host = "ws://" + config_file["url"]
        topics_to_subscribe = config_file["topics_to_subscribe"]

        self.logger = self.get_logger()

        self.socket_client = roslibpy.Ros(host=host)

        self.socket_client.run(5)
        if not self.socket_client.is_connected:
            self.get_logger().error(
                "Could not connect to server. Make sure rosbridge is running on the robot"
            )
            exit(1)

        self.ros_publishers_dict = {}
        self.socket_subscribers_dict = {}

        # Attributes to measure metrics
        self.done = False
        self.start_time = time()
        self.start_bytes = net_io_counters().bytes_recv
        self.cum_latency_dict = {}
        self.received_messages_dict = {}

        for topic in list(topics_to_subscribe.keys()):
            self.cum_latency_dict[topic] = 0.0

        for topic in list(topics_to_subscribe.keys()):
            self.received_messages_dict[topic] = 0

        for topic in list(topics_to_subscribe.keys()):
            self.ros_publishers_dict[topic] = self.create_publisher(
                msg_type=eval(topics_to_subscribe[topic]["type"]),
                topic=topic,
                qos_profile=qos_profile_sensor_data,
            )
            self.socket_subscribers_dict[topic] = roslibpy.Topic(
                ros=self.socket_client,
                name=topic,
                message_type=ros2_to_ros1_msg_string(
                    ros2_msg_string=topics_to_subscribe[topic]["type"]
                ),
                compression=topics_to_subscribe[topic]["compression"],
            )
            self.socket_subscribers_dict[topic].subscribe(
                # WTF: https://stackoverflow.com/questions/7546285/creating-lambda-inside-a-loop
                lambda message, topic=topic, type=type: self.publish_ros_message(
                    message,
                    topic,
                    topics_to_subscribe[topic]["type"].replace(".", "/"),
                )
            )

        th_results = Thread(target=self.print_test_result)
        th_results.start()

    def print_test_result(self):
        """! Function to print the benchmark results."""
        while not self.done:
            delta_time = time() - self.start_time
            if delta_time > 60.0:

                # Calculate the downloaded kb
                download_kb = (net_io_counters().bytes_recv - self.start_bytes) / 1024.0

                self.logger.info(
                    f"Avg. download speed is: {download_kb / delta_time:8.2f} KiB/s"
                )

                for topic in list(self.cum_latency_dict.keys()):
                    self.logger.info(
                        f"For topic {topic}, avg. latency is: {(self.cum_latency_dict[topic] * 1000) / (self.received_messages_dict[topic] -1):3.2f} ms."
                    )

                # Stop iterating in the thread
                self.done = True
            sleep(1.0)

    def publish_ros_message(self, message, topic, type):
        message = message_converter.convert_dictionary_to_ros_message(type, message)
        self.received_messages_dict[topic] += 1
        self.cum_latency_dict[topic] += abs(
            time() - self.stamp_to_seconds(message.header.stamp)
        )
        self.ros_publishers_dict[topic].publish(message)

    def stamp_to_seconds(self, stamp):
        return stamp.sec + stamp.nanosec * 10**-9


def main(args=None) -> None:

    rclpy.init(args=args)

    # Execute work and block until the context associated with the
    # executor is shutdown.
    rosbridge_client = RosBridgeClient()

    # Execute work and block until the context associated with the
    # executor is shutdown. Callbacks will be executed by the provided
    # executor.
    rclpy.spin(rosbridge_client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rosbridge_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
