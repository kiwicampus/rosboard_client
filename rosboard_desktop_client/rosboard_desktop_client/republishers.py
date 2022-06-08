#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
"""
Code Information:
    Code Information:
    Maintainer: Eng. Pedro Alejandro Gonzalez B
	Mail: pedro@kiwibot.com
"""

# =============================================================================
import importlib
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import logging

from sensor_msgs.msg import Image, PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from rclpy_message_converter.message_converter import convert_dictionary_to_ros_message

import numpy as np

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

    def publish(self, msg: any) -> None:
        """!
        Function to publish ros messages on the specified topic
        @param msg (any) The message to publish
        """
        if not isinstance(msg, self.topic_class):
            self.logger.error(
                f"tried to publish a message of type {type(msg).__name__} while a publisher was created for type {self.topic_class_name}"
            )
            return
        if self.publisher is None:
            self.logger.error(f"tried to publish message but no node was provided")
            return
        self.publisher.publish(msg)

    def parse_and_publish(self, rosboard_data: list) -> None:
        """!
        Function to parse a message from rosboard data and publish it
        @param rosboard_data (list) the data from rosboard. the binary fields must have already been decoded
        """
        self.publish(self.parse_message(rosboard_data))

    def parse_message(self, rosboard_data: list) -> list:
        """!
        Function to convert the rosboard data to a ROS message. If a field in the message
        is not contained in the rosboard data it will be left empty on the ROS message
        @param rosboard_data (list) The rosboard data, the binary fields must have already been decoded
        @return any The ROS message
        """
        # Make use of rclpy message converter. with strict_mode=False fields not present
        # in the dictionary will be left empty on the ROS message
        message = convert_dictionary_to_ros_message(
            self.topic_class_name.replace(".", "/"), rosboard_data[1], strict_mode=False
        )
        return message


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
            base_occupancy_grid.info.width / image_bytes.shape[1]
        )
        # adjust size for rosboard subsampling
        base_occupancy_grid.info.width = image_bytes.shape[1]
        base_occupancy_grid.info.height = image_bytes.shape[0]
        occupancy_grid_array = image_bytes.astype(np.int8)

        base_occupancy_grid.data = (
            occupancy_grid_array.flatten().astype(np.int8).tolist()
        )
        return base_occupancy_grid

    @classmethod
    def supported_msg_types(self) -> list:
        """!
        Get the message types supported by the OccupancyGridPublisher
        @return list a list with all the supported message types
        """
        return ["nav_msgs/msg/OccupancyGrid"]


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


class PublisherManager:
    """!
    Class to manage available publishers and return the appropriate
    publisher for each message type
    """

    available_publishers = [
        ImagePublisher,
        PointCloudPublisher,
        OccupancyGridPublisher,
        LaserScanPublisher,
    ]
    default_publisher = GenericPublisher

    @classmethod
    def getDefaultPublisherForType(self, topic_type: str) -> GenericPublisher:
        """! Function to get a publisher class for a given message type
        @param topic_type (str) the message type using the rosboard formar. Ex: nav_msgs/msg/Path
        @return Publisher class with the most appropriate publisher for the given topic type
        """
        supported_publishers = list(
            filter(
                lambda publisher: topic_type in publisher.supported_msg_types(),
                self.available_publishers,
            )
        )
        # In case no special publishers are found for the topic type, use a generic publisher
        if len(supported_publishers):
            return supported_publishers[0]
        return self.default_publisher
