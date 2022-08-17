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
import os
import re
import yaml
import rclpy
from sys import exit
from time import time, sleep
from threading import Thread
from icmplib import ping, NameLookupError
from psutil import net_io_counters
from socket import gaierror

from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from rosboard_client.republishers import PublisherManager

from rosboard_client.streamers import GenericStreamer

from rosboard_client.networking import RosboardClient


class RosboardBenchmark(Node):

    URL_RE = "^((?P<scheme>[^:/?#]+):(?=//))?(//)?(((?P<login>[^:]+)(?::(?P<password>[^@]+)?)?@)?(?P<host>[^@/?#:]*)(?::(?P<port>\d+)?)?)?(?P<path>[^?#]*)(\?(?P<query>[^#]*))?"

    def __init__(self):
        """!
        Node object that parses a yaml file to create rosboard subscriptions and
        republishing remote topics on the local machine
        """
        Node.__init__(self, node_name="rosboard_yaml_client")
        self.logger = self.get_logger()

        # Read and parse config file
        config_file_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "benchmark_config.yaml"
        )
        with open(config_file_path, "r") as stream:
            config_dict = yaml.safe_load(stream)
        self.host = config_dict["url"]
        topics_to_subscribe = config_dict["topics"]
        topics_to_stream = config_dict["topics_to_stream"]

        self.client = RosboardClient(host=self.host, connection_timeout=5)

        match = re.search(RosboardYamlNode.URL_RE, self.host)
        self.host_addr = match.group("host")

        # Topic handler list attribute
        self.topic_handlers_list = []

        # Subscribe to rosboard topics
        for topic in topics_to_subscribe:

            # Try to create a topic handler
            try:
                self.topic_handlers_list.append(TopicHandler(topic, self.client, self))
                self.logger.info(f"Subscribed to {topic}")
            except Exception as e:
                print(e)

        # Subscribe to local topics
        for topic in topics_to_stream:
            GenericStreamer(self, self.client, topic)

        # Measurement attributes
        self.start_time = time()
        self.done = False
        self.roundtrip_samples = 0
        self.cum_roundtrip = 0.0

        self.start_download_bytes = net_io_counters().bytes_recv

        # Start measurement threads
        th_print = Thread(target=self.print_results)
        th_print.start()

    def print_results(self):
        """! Function to print the benchmark results."""
        while not self.done:
            delta_time = time() - self.start_time
            if delta_time > 60.0:

                # Calculate the downloaded kb
                download_kb = (
                    net_io_counters().bytes_recv - self.start_download_bytes
                ) / 1024.0

                self.logger.info(
                    f"Avg. download speed is: {download_kb / delta_time:8.2f} KiB/s"
                )

                for th in self.topic_handlers_list:
                    if th.n_msgs != 0:
                        self.logger.info(
                            f"For topic {th.topic_name}, avg. latency is: {(th.cum_latency * 1000.0) / (th.n_msgs-1):3.2f} ms."
                        )
                self.done = True
            sleep(1.0)


class TopicHandler(object):
    def __init__(self, topic_name: str, client: RosboardClient, node: Node):

        # Store topic name and client instance
        self.topic_name = topic_name
        self.client = client
        self.node = node

        # Attributes to store the number of messages, first message time stamp
        self.n_msgs = 0
        self.ref_time = 0.0
        self.ref_timestamp = 0.0
        self.cum_latency = 0.0

        # Check if topic is available
        if not self.client.is_topic_available(self.topic_name):
            raise Exception(f"Topic {self.topic_name} is not available in server")

        # Get topic type
        topic_type = self.client.get_topic_type(self.topic_name)

        # Define a republisher class for the topic with the given type
        republisher_class = PublisherManager.getDefaultPublisherForType(topic_type)

        # Define the republisher for the topic with its type
        self.republisher = republisher_class(self.node, self.topic_name, topic_type)

        # Create the topic subscription for the topic
        self.client.create_socket_subscription(
            topic_type, self.topic_name, self.topic_callback
        )

    def topic_callback(self, msg: list):

        # Check if topic has header
        if "header" in msg[1].keys():

            # Get the timestamp
            timestamp = self.timestamp_to_secs(msg[1]["header"]["stamp"])

            # Get the time difference
            latency = abs(time() - timestamp)

            # Store the cumulative latency
            self.cum_latency += latency

        self.republisher.parse_and_publish(msg)
        self.n_msgs += 1

    def timestamp_to_secs(self, header_stamp: dict) -> float:
        """! Convert a header timestamp to a float value.

        @param header_stamp "dict" header with fields using for converting the
        timestamp to seconds.
        @return "float" value with the header timestamp.
        """
        return header_stamp["sec"] + header_stamp["nanosec"] * (10**-9)


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
