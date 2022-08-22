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
from rosboard_client.ros.republishers import (
    GenericPublisher,
    ImagePublisher,
    LaserScanPublisher,
    OccupancyGridPublisher,
    PointCloudPublisher,
)


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
