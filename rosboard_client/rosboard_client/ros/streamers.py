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

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy_message_converter.message_converter import convert_ros_message_to_dictionary
from rosboard_client.client.networking import RosboardClient


class GenericStreamer:
    def __init__(
        self,
        parent_node: Node,
        parent_rosboard_client: RosboardClient,
        topic_name: str,
        topic_type: str = None,
    ) -> None:
        """!
        Class to subscribe to a local ROS topic and stream its messages to a
        rosboard server
        @param parent_node (Node) the node object to create subscriptions
        @param parent_rosboard_client (RosboardClient) the socket client to send messages
        @param topic_name (str) the topic name
        @param topic_type (str, optional) The topic type i.e: sensor_msgs/msg/NavSatFix. If not provided
        the class will get the topic type from one of the message publishers. Defaults to None.
        """
        self.parent_node = parent_node
        self.parent_rosboard_client = parent_rosboard_client
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.subscriber = None
        self.logger = self.parent_node.get_logger()
        self.create_subscription(topic_name, topic_type)
        self.logger.info(f"Subscription to topic {topic_name} created successfully")

    def destroy_subscription(self):
        """! Destroy the subscription to the topic if it exists. If the
        subscription does not exists and is attempted to be destroyed, a
        warning message will be presented.

        Raises:
            Exception: in case that the subscription exists and can not be destroyed.
        """
        self.parent_rosboard_client.destroy_socket_publisher(self.topic_name)
        if self.subscriber is not None:
            if not self.parent_node.destroy_subscription(self.subscriber):
                raise Exception(
                    f"Could not destroy subscription to {self.topic_name} topic!"
                )
        else:
            self.parent_node.get_logger().warn(
                f"Destroying inexistent subscription to {self.topic_name} topic!"
            )

    def create_subscription(self, topic_name: str, topic_type: str = None) -> None:
        """!
        Function to create a subscription to a local ROS topic. If topic type is not provided
        it will try to guess it from available publishers of the topic
        @param topic_name (str) the topic name
        @param topic_type (str, optional) the topic type. If not provided it will
        try to guess it from available publishers of the topic. Defaults to None.
        Raises:
            Exception: In case the topic type is not provided and there are no publishers for the given topic
            ModuleNotFoundError: in case the topic type cannot be imported. i.e: a custom message without a sourced workspace
            ValueError: in case the topic type is invalid not having the standard structure i.e: nav_msgs/msg/Path
        """
        # get topic type and qos
        topic_info = self.parent_node.get_publishers_info_by_topic(topic_name)
        if not topic_info:
            if topic_type is None:
                raise Exception(
                    f"topic type for topic {topic_name} was not provided and cannot get its type because it is not being published"
                )
            self.logger.info(
                f"topic {topic_name} has no publishers, cannot get its qos and will subscribe with sensor data qos"
            )
            qos = qos_profile_sensor_data
            topic_class_name = topic_type
        else:
            qos = topic_info[0].qos_profile
            # use topic type from the first publisher if not provided
            topic_class_name = (
                topic_info[0].topic_type if topic_type is None else topic_type
            )
        self.topic_type = topic_class_name

        # dynamically import the message class
        try:
            # get module substring and message type. Ex: sensor_msgs.msg, Image from sensor_msgs.msg.Image
            msg_module, _, msg_class_name = topic_class_name.replace(
                "/", "."
            ).rpartition(".")
            if not msg_module.endswith(".msg"):
                msg_module = msg_module + ".msg"
            topic_class = getattr(importlib.import_module(msg_module), msg_class_name)
        except ModuleNotFoundError:
            raise ModuleNotFoundError(
                f"Could not import {topic_class_name}. Is your workspace sourced?"
            )
        except ValueError:
            raise ValueError(f"Invalid message type: {topic_class_name}")

        self.subscriber = self.parent_node.create_subscription(
            msg_type=topic_class,
            topic=topic_name,
            callback=self.stream_message,
            qos_profile=qos,
        )

    def stream_message(self, msg: any) -> None:
        """!
        Function to send a ROS message to a remote rosboard server
        @param msg (any) the ROS message
        """
        msg_dict = convert_ros_message_to_dictionary(msg)
        msg_dict["_topic_name"] = self.topic_name
        msg_dict["_topic_type"] = self.topic_type
        self.parent_rosboard_client.send_ros_message(msg_dict)
