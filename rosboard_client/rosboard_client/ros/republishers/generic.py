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
import importlib
import logging

from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


tf_static_qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


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
            ValueError: When the message type is not found within the message module
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

        # dynamically import the message class
        try:
            # get module substring and message type. Ex: sensor_msgs.msg, Image from sensor_msgs.msg.Image
            msg_module, _, msg_class_name = topic_class_name.replace(
                "/", "."
            ).rpartition(".")
            if not msg_module.endswith(".msg"):
                msg_module = msg_module + ".msg"
            self.topic_class = getattr(
                importlib.import_module(msg_module), msg_class_name
            )
        except ModuleNotFoundError:
            raise ModuleNotFoundError(
                f"Could not import {topic_class_name}. Is your workspace sourced?"
            )
        except ValueError:
            raise ValueError(f"Invalid message type: {topic_class_name}")

        self.logger = parent_node.get_logger()

        # Create the message publisher
        qos_profile = tf_static_qos if topic_name == "/tf_static" else 1
        self.publisher = parent_node.create_publisher(
            msg_type=self.topic_class, topic=topic_name, qos_profile=qos_profile
        )
