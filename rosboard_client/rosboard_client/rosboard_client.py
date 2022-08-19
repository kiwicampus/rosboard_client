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

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rosboard_client.client import RosboardClient
from rosboard_client.ros import GenericStreamer, PublisherManager


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
        topics_to_stream = config_dict["topics_to_stream"]

        self.client = RosboardClient(host=host, connection_timeout=5)

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
            self.logger.info(f"subscribed to {topic} of type {topic_type}")

        # Subscribe to local topics
        for topic in topics_to_stream:
            GenericStreamer(self, self.client, topic)


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
