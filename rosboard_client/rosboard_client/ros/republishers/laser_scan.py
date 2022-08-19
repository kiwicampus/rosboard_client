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

import numpy as np
from rclpy.node import Node
from rclpy_message_converter.message_converter import convert_dictionary_to_ros_message
from rosboard_client.ros.republishers.generic import GenericPublisher
from sensor_msgs.msg import LaserScan


class LaserScanPublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args, **kwargs) -> None:
        """!
        Class to parse ROS LaserScans from rosboard laserscan data. Inherits from the GenericPublisher
        Refer to https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py
        to see how data is encoded and compressed
        @param parent_node (Node) A node object to create the PointCloud2 publisher
        @param topic_name (str) The name of the PointCloud2 topic to republish messages
        """
        super().__init__(parent_node, topic_name, "sensor_msgs.msg.LaserScan")

    def parse_message(self, rosboard_data: list) -> LaserScan:
        """!
        Overrides the parse_message function from the GenericPublisher to get a ROS
        LaserScan from the rosboard decoded data
        @param rosboard_data (list) The rosboard data, the binary fields must have already been decoded
        @return sensor_msgs.msg.LaserScan The ROS LaserScan message
        """
        binary_ranges = rosboard_data[1]["_ranges_uint16"]["points"]
        binary_intensities = rosboard_data[1]["_intensities_uint16"]["points"]
        rmin, rmax = rosboard_data[1]["_ranges_uint16"]["bounds"]
        imin, imax = rosboard_data[1]["_intensities_uint16"]["bounds"]
        # https://stackoverflow.com/questions/53971620/cant-modify-numpy-array
        ranges_array = np.frombuffer(binary_ranges, np.uint16).astype(np.float32).copy()
        # zero or nan points are encoded by rosboard with 65535
        # see: https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py#L351
        invalid_idxs = ranges_array == 65535.0
        intensities_array = (
            np.frombuffer(binary_intensities, np.uint16).astype(np.float32).copy()
        )
        # Rosboard sends points scaled between 0 and 65534 where 0 maps to xmin and 65534 maps to xmax
        # 65535 is invalid value (nan/-inf/inf)
        ranges_array = self.scale_back_array(ranges_array, rmin, rmax)
        intensities_array = self.scale_back_array(intensities_array, imin, imax)
        ranges_array[invalid_idxs] = 0.0
        base_laser_msg = convert_dictionary_to_ros_message(
            "sensor_msgs/msg/LaserScan", rosboard_data[1], strict_mode=False
        )
        base_laser_msg.ranges = ranges_array.flatten().tolist()
        base_laser_msg.intensities = intensities_array.flatten().tolist()
        return base_laser_msg

    def scale_back_array(
        self, array: np.ndarray, min_val: float, max_val: float
    ) -> np.ndarray:
        """!
        Scale array back to meters according to the rosboard. Rosboard sends
        points scaled between 0 and 65534 where 0 maps to xmin and 65535 maps to xmax
        65535 is invalid value (nan/-inf/inf)
        @param array (np.ndarray) the array containing the scaled data
        @param min_val (float) the min value in meters
        @param max_val (float) the max value in meters
        @return np.ndarray the array with the values in meters
        """
        return (array / 65534.0) * (max_val - min_val) + min_val

    @classmethod
    def supported_msg_types(self):
        """!
        Get the message types supported by the LaserScanPublishers
        @return list a list with all the supported message types
        """
        return ["sensor_msgs/msg/LaserScan"]
