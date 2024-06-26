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

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy_message_converter.message_converter import convert_dictionary_to_ros_message
from rosboard_client.ros.republishers.generic import GenericPublisher


class OccupancyGridPublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args, **kwargs) -> None:
        """!
        Class to parse ROS OccupancyGrids from rosboard data. Inherits from the GenericPublisher
        This only works with our fork, because encoding for occupancy grids was changed to PNG
        Refer to https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py
        to see how data is encoded and compressed
        @param parent_node (Node) A node object to create the OccupancyGrid publisher
        @param topic_name (str) The name of the OccupancyGrid topic to republish messages
        """
        super().__init__(parent_node, topic_name, "nav_msgs.msg.OccupancyGrid")

    def parse_message(self, rosboard_data) -> OccupancyGrid:
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
        base_occupancy_grid.info.width = image_bytes.shape[1]
        base_occupancy_grid.info.height = image_bytes.shape[0]

        occupancy_grid_array = image_bytes.astype(np.int8)

        base_occupancy_grid.data = (
            occupancy_grid_array.flatten("C").astype(np.int8).tolist()
        )
        return base_occupancy_grid

    @classmethod
    def supported_msg_types(self) -> list:
        """!
        Get the message types supported by the OccupancyGridPublisher
        @return list a list with all the supported message types
        """
        return ["nav_msgs/msg/OccupancyGrid"]
