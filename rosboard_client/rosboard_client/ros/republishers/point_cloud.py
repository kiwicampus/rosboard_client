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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudPublisher(GenericPublisher):
    def __init__(self, parent_node: Node, topic_name: str, *args, **kwargs) -> None:
        """!
        Class to parse ROS PointCloud2 from rosboard pointcloud data. Inherits from the GenericPublisher
        Keep in mind that rosboard only sends xyz data, removing all the other PointFields
        Refer to https://github.com/kiwicampus/rosboard/blob/main/rosboard/compression.py
        to see how data is encoded and compressed
        @param parent_node (Node) A node object to create the PointCloud2 publisher
        @param topic_name (str) The name of the PointCloud2 topic to republish messages
        """
        super().__init__(parent_node, topic_name, "sensor_msgs.msg.PointCloud2")

    def parse_message(self, rosboard_data: list) -> PointCloud2:
        """!
        Overrides the parse_message function from the GenericPublisher to get a ROS
        PointCloud from the rosboard decoded data
        @param rosboard_data (list) The rosboard data, the binary fields must have already been decoded
        @return sensor_msgs.msg.PointCloud2 The ROS PointCloud2 message
        """
        binary_data = rosboard_data[1]["_data_uint16"]["points"]
        xmin, xmax, ymin, ymax, zmin, zmax = rosboard_data[1]["_data_uint16"]["bounds"]
        points_array = np.frombuffer(binary_data, np.uint16)
        # Points are contained in the rosboard array sequentially as x-y-z, x-y-z ...
        # Thus a 3 channel np array is created where each channel contains the points
        # in each coordinate.
        points_array = points_array.reshape(points_array.shape[0] // 3, 3).astype(
            np.float32
        )
        # Rosboard sends points scaled between 0 and 65535 where 0 maps to xmin and 65535 maps to xmax
        points_array[:, 0] = self.scale_back_array(points_array[:, 0], xmin, xmax)
        points_array[:, 1] = self.scale_back_array(points_array[:, 1], ymin, ymax)
        points_array[:, 2] = self.scale_back_array(points_array[:, 2], zmin, zmax)
        base_pc_msg = convert_dictionary_to_ros_message(
            "sensor_msgs/msg/PointCloud2", rosboard_data[1], strict_mode=False
        )
        # avoided for efficiency reasons
        # xyz_pc = point_cloud2.create_cloud_xyz32(header=base_pc_msg.header, points=points_array)
        xyz_pc = point_cloud2.create_cloud_xyz32(header=base_pc_msg.header, points=[])
        xyz_pc.width = points_array.shape[0]
        xyz_pc.point_step = 12
        xyz_pc.row_step = xyz_pc.width * xyz_pc.point_step
        # Since the array is already made by floats, it can be directly turned to bytes and copied
        # to the message data field
        xyz_pc.data = points_array.tobytes()
        return xyz_pc

    def scale_back_array(
        self, array: np.ndarray, min_val: float, max_val: float
    ) -> np.ndarray:
        """!
        Scale array back to meters according to the rosboard. Rosboard sends
        points scaled between 0 and 65535 where 0 maps to xmin and 65535 maps to xmax
        @param array (np.ndarray) the array containing the scaled data
        @param min_val (float) the min value in meters
        @param max_val (float) the max value in meters
        @return np.ndarray the array with the values in meters
        """
        return (array / 65535.0) * (max_val - min_val) + min_val

    @classmethod
    def supported_msg_types(self) -> list:
        """!
        Get the message types supported by the PointCloudPublisher
        @return list a list with all the supported message types
        """
        return ["sensor_msgs/msg/PointCloud2"]
