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

import base64

import cv2
import numpy as np
import simplejpeg


def jpeg_bgr_decode(binary_data: str) -> np.ndarray:
    """!
    Function to get a 3 channel np.ndarray image representation from a jpeg
    base64 encoded string
    @param binary_data (str) the base64 encoded jpeg string
    @return np.ndarray the 3 channel RGB image data
    """
    return simplejpeg.decode_jpeg(base64.b64decode(binary_data), colorspace="bgr")


def png_gray_decode(binary_data: str) -> np.ndarray:
    """!
    Function to get a single channel np.ndarray image representation from a png
    base64 encoded string
    @param binary_data (str) the base64 encoded png string
    @return np.ndarray the single channel grayscale image data
    """
    return cv2.imdecode(
        np.fromstring(base64.b64decode(binary_data), dtype="uint8"),
        cv2.IMREAD_UNCHANGED,
    )


def default_decode(binary_data: str) -> bytes:
    """!
    Function to decode a base64 encoded string back to bytes
    @param binary_data (str) the base64 encoded string
    @return bytes the bytes
    """
    return base64.b64decode(binary_data)


class RosboardDecoder:
    """!
    Class to take in rosboard message payloads as dictionaries and decode/decompress
    the binary/encoded fields. The variable `decoders` contains the fields and type
    of decoding for each message.
    """

    decoders = {
        "sensor_msgs/msg/Image": {"_data_jpeg": jpeg_bgr_decode},
        "sensor_msgs/msg/CompressedImage": {"_data_jpeg": jpeg_bgr_decode},
        "nav_msgs/msg/OccupancyGrid": {"_data_jpeg": png_gray_decode},
        "sensor_msgs/msg/PointCloud2": {"_data_uint16.points": default_decode},
        "sensor_msgs/msg/LaserScan": {
            "_ranges_uint16.points": default_decode,
            "_intensities_uint16.points": default_decode,
        },
    }

    def decode_field(self, dictionary: dict, key: str, decode_function) -> dict:
        """!
        Function to decode a binary field on a dictionary by its key and decode function.
        Keys can be passed recursively as `key1.key2` to change fields in a dictionary contained
        inside another.
        @param dictionary (dict) The dictionary containing the field to decode
        @param key (str) The key with the binary data. If the data is contained in a dictionary within
        a dictionary, keys can be passed recursively as `key1.key2`
        @param decode_function (function) The decode function to apply to the binary data on the field
        @return dict The same dictionary with the specified field updated containing the decoded data
        """
        if "." in key:
            new_key = key[key.find(".") + 1 :]
            old_key = key[: key.find(".")]
            dictionary[old_key] = self.decode_field(
                self, dictionary[old_key], new_key, decode_function
            )
        else:
            dictionary[key] = decode_function(dictionary[key])
        return dictionary

    @classmethod
    def decode_binary_fields(self, rosboard_data: list) -> list:
        """!
        Function to decode the binary fields from a rosboard raw message as a list.
        Rosboard messages are lists composed by two elements, the first being a character
        identifying the type of data contained in the message and the second the data itself
        @param rosboard_data (list) the rosboard message after dumping it from json.
        @return list The same rosboard message but with the binary fields containing the decoded
        data
        """
        msg_type = rosboard_data[1]["_topic_type"]
        if not msg_type in self.decoders.keys():
            return rosboard_data

        for field, decoder in self.decoders[msg_type].items():
            rosboard_data[1] = self.decode_field(self, rosboard_data[1], field, decoder)
        return rosboard_data
