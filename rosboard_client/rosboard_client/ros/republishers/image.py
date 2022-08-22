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

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy_message_converter.message_converter import convert_dictionary_to_ros_message
from rosboard_client.ros.republishers.generic import GenericPublisher
from sensor_msgs.msg import Image


class ImagePublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args, **kwargs) -> None:
        """!
        Class to parse ROS images from rosboard image data. Inherits from the GenericPublisher
        Refer to https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py
        to see how data is encoded and compressed
        @param parent_node (Node) A node object to create the image publisher
        @param topic_name (str) The name of the image topic to republish messages
        """
        super().__init__(parent_node, topic_name, "sensor_msgs.msg.Image")
        self.bridge = CvBridge()

    def parse_message(self, rosboard_data: list) -> Image:
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
