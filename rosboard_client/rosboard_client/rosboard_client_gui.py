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
    Code Information: PyQt5 user interface for the rosboard client.
    Maintainer: Nicolas Rocha Pacheco
	Mail: nicolas.rocha@kiwibot.com
"""

import bisect

# =============================================================================
import os
import re
import sys
from functools import partial
from socket import AF_INET, SOCK_STREAM, gaierror, socket
from threading import Thread
from time import sleep, time
from typing import Tuple

import PyQt5
import rclpy
from icmplib import NameLookupError, ping
from psutil import cpu_percent, net_io_counters
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rosboard_client.client import RosboardClient
from rosboard_client.ros import GenericStreamer, PublisherManager


class TopicHandler:
    def __init__(
        self,
        topic_name: str,
        client: RosboardClient,
        node: Node,
        alpha: float = 0.1,
        state_sleep: float = 0.25,
    ):
        """! Class to handle the topic subscription and statistics.

        This class provides a mechanism to connect a topic to the rosboard
        client and to generate statistics measurements for topics such
        as the current latency and rate in which messages are received.

        @param topic_name "str" name of the topic being handled.
        @param client "RosboardClient" client for the topic.
        @param node "Node" ROS node instance to communicate with ROS.
        @param alpha "float" (0.1 by default) alpha parameter for calculating
        the moving average of values.
        @param state_sleep "float" (0.25 by default) the sleep time in which
        the topic state is checked.
        """
        # Initialize attributes
        self.node = node
        self.client = client
        self.topic_name = topic_name
        self.has_header = False

        # Variables for period
        self.last_period = 0.0
        self.last_period_var = 0.0

        # Variables for latency
        self.last_latency = 0.0
        self.last_latency_var = 0.0

        self.rate = 0.0
        self.alpha = alpha
        self.state = "NO_DATA"
        self.state_sleep = state_sleep

        # Get the topic message type and create republisher
        message_type = client.get_topic_type(topic_name)
        default_publisher = PublisherManager.getDefaultPublisherForType(message_type)
        self.republisher = default_publisher(
            parent_node=self.node,
            topic_name=self.topic_name,
            topic_class_name=message_type,
        )
        self.client.create_socket_subscription(
            message_type, self.topic_name, self.topic_callback
        )

        # Define member variables to store values
        self.n_msgs = 0
        self.t_start = time()
        self.t_last_msg = None
        self.running = True

        # Create timers to run auxiliary functions
        self.th_state = Thread(target=self.define_node_state, daemon=True)
        self.th_state.start()

    def __del__(self):
        """! Class destructor for TopicHandler.
        Used to stop threads from continuous execution.
        """
        self.running = False

    def destroy_subscription(self):
        """! Destroy the subscription to the topic."""
        self.node.get_logger().info(f"Closing connection for {self.topic_name}")
        self.client.destroy_socket_subscription(self.topic_name)
        self.node.destroy_publisher(self.republisher.publisher)
        self.running = False

    def define_node_state(self):
        """! Function that defines the node state.

        The state depends on the time in which the latest message was received
        and the frequency in which messages are received. If more than five
        (5) seconds have passed since the latest received message, the topic
        handler will enter into 'NO_DATA' state. If the rate in which messages
        are received drop from 80% of the historic average rate, the topic
        handler will enter into 'DELAY' state. If neither of this conditions
        is achieved, the topic handler will be in a 'NORMAL' state.
        """
        while self.running:
            if self.n_msgs > 1:
                t_current = time()
                mean = (t_current - self.t_start) / self.n_msgs
                if t_current - self.t_last_msg > 5.0:
                    self.state = "NO_DATA"
                elif abs(self.last_period - mean) > 2 * (self.last_period_var**0.5):
                    self.state = "DELAY"
                else:
                    self.state = "NORMAL"
            sleep(self.state_sleep)

    def calculate_average_and_var(
        self, current_value: float, last_value: float, last_var: float
    ) -> Tuple[float, float]:
        """! Calculate the exponentially weighted moving average.

        This function calculates the exponentially weighted moving average
        (EWMA) for a given list of values. The function does not expect the
        list to have a fixed value, although it will use the predefined
        'ALPHA' constant, which might depend on the expected number of items
        of the list.

        @param current_value "float" current value.
        @param last_value "float" last EMWA value.
        @return "float" with the average value. Return 0.0 if time_list has
        no values.
        """
        if self.n_msgs == 0:
            current_var = (1.0 - self.alpha) * (self.alpha * current_value**2)
            return current_value, current_var
        else:
            ewma = self.alpha * current_value + (1.0 - self.alpha) * last_value
            delta = current_value - last_value
            current_var = (1.0 - self.alpha) * (last_var + self.alpha * delta**2)
            return ewma, current_var

    def topic_callback(self, msg: list):
        """! Topic callback function for incoming rosboard messages.

        This function processes the incoming messages in order to provide the
        stats of the current topic handler. It adds one unit to the count of
        received messages. Then, it captures the current time, that is then used
        to
        """
        t_current = time()
        if self.n_msgs > 0:
            if self.has_header:
                t_send = self.timestamp_to_secs(msg[1]["header"]["stamp"])
                current_latency = t_current - t_send
                (
                    self.last_latency,
                    self.last_period_var,
                ) = self.calculate_average_and_var(
                    current_latency, self.last_latency, self.last_period_var
                )
            current_period = t_current - self.t_last_msg
            self.last_period, self.last_period_var = self.calculate_average_and_var(
                current_period, self.last_period, self.last_period_var
            )
        else:
            self.has_header = "header" in msg[1].keys()

        # Always execute this code block
        self.t_last_msg = t_current
        self.republisher.parse_and_publish(msg)
        self.n_msgs += 1

    def timestamp_to_secs(self, header_stamp: dict) -> float:
        """! Convert a header timestamp to a float value.

        @param header_stamp "dict" header with fields using for converting the
        timestamp to seconds.
        @return "float" value with the header timestamp.
        """
        return header_stamp["sec"] + header_stamp["nanosec"] * (10**-9)

    def get_topic_stats(self) -> list:
        """! Calculate and return the topic statistics.

        A simple process is followed to calculate the topic statistics: an
        average for the time between messages is calculated using EMWA. This
        value is inverted to

        @return "list" includes three (3) values: the rate in which the latest
        messages were received; the latency if the message includes a header, and
        a boolean indicating if the message has header.
        """
        rate = 1.0 / self.last_period if self.last_period != 0.0 else 0.0
        return [rate, self.last_latency]


class RosboardClientGui(QMainWindow):

    # Regular expression to capture network scheme, login, password, host,
    #  port, path and query from an URL.
    URL_RE = "^((?P<scheme>[^:/?#]+):(?=//))?(//)?(((?P<login>[^:]+)(?::(?P<password>[^@]+)?)?@)?(?P<host>[^@/?#:]*)(?::(?P<port>\d+)?)?)?(?P<path>[^?#]*)(\?(?P<query>[^#]*))?"

    # Dictionary to map schemes to ports
    PORTS = {"DEFAULT": 80, "http": 80, "https": 443}

    # List to store the valid protocols.
    VALID_PROTOCOLS = ["ws", "wss", "http", "https", "tcp"]

    def __init__(
        self,
        cpu_timer_to: float = 250,
        rtt_timer_to: float = 1000,
        download_timer_to: float = 250,
        global_stats_timer_to: float = 250,
        topic_stats_timer_to: float = 250,
        topic_upd_timer_to: float = 500,
    ):
        """! Main window class for the rosboard client GUI.

        The main window class will organize a set of widgets in which the
        functionalities are implemented. There are four main widgets in the
        rosboard client GUI: 'network connection', 'stats', 'topics list' and
        'topics panel'. The 'network connection' widget allows the user to
        input an URL and connect to the server. The 'stats' widget is in
        charge of presenting global statistics to the user. The 'topics list'
        presents the available topics in the server. Finally, the 'topics
        panel' widget presents the handled topics from the server with their
        stats.

        This class contains timers that handle some of the interface
        functionalities:
            1. 'cpu_usage_timer' (callback every 250ms): this timer retrieves
            the CPU usage percentage.
            2. 'roundtrip_timer' (callback every 1000ms): this timer gets the
            roundtrip time value to the server when connected.
            3. 'download_speed_timer' (callback every 250ms): calculates the
            host download speed.
            4. 'stats_timer' (callback every 250ms): updates the global
            statistics in the interface.
            5. 'topic_stats_timer' (callback every 250ms): updates the handled
            topic statistics.
            6. 'restore_timer' (callback every 5000ms): checks if the interface
            needs to be restored. The interface is restored when the connection
            to the server is lost and gained again.
            7. 'topic_upd_timer' (callback every 500ms): timer to update the
            available topics in the topics list widget.

        @param cpu_timer_to "float" timeout value in milliseconds for the
        'cpu_usage_timer' timer. It has a default value of 250.
        @param rtt_timer_to "float" timeout value in milliseconds for the
        'roundtrip_timer' timer. It has a default value of 1000.
        @param download_timer_to "float" timeout value in milliseconds for
        the 'download_speed_timer' timer. It has a default value of 250.
        @param global_stats_timer_to "float" timeout value in milliseconds
        for the 'stats_timer' timer. It has a default value of 250.
        @param topic_stats_timer_to "float" timeout value in milliseconds for
        the 'topic_stats_timer' timer. It has a default value of 250.
        @param topic_upd_timer_to "float" timeout value in milliseconds for
        the 'topic_upd_timer' timer. It has a default value of 500.
        """
        super(QMainWindow, self).__init__()
        self.setMinimumSize(650, 400)

        # Load stylesheet
        stylesheet_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "resources", "rosboard.qss"
        )
        with open(stylesheet_path, "r") as ss_file:
            self.setStyleSheet(ss_file.read())

        # Set the icon
        icon_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "resources", "icon.png"
        )
        self.setWindowIcon(QIcon(icon_path))

        # Start the ROS node
        self.node = Node("rosboard_client_gui")

        # Initialize network attributes
        self.reset_network_attributes()

        # Main window configurations
        self.setWindowTitle("Rosboard Client GUI")
        self.setAccessibleName("")

        # Initialize custom widgets
        self.connection_widget = ConnectionWidget(self)
        self.stats_widget = StatusWidget(self)
        self.server_topics_list_wg = TopicsListWidget(self, self.add_topic_to_panel)
        self.server_topics_panel_wg = TopicsPanelWidget(
            self, ExtendedTopicWidget, self.move_from_server_panel_to_list
        )
        self.client_topics_list_wg = TopicsListWidget(self, self.add_client_to_panel)
        self.client_topics_panel_wg = TopicsPanelWidget(
            self, TopicWidget, self.move_from_client_panel_to_list
        )

        # List to store streamers and client topics
        self.client_topics = []
        self.streamers = []

        # List to store recently removed client to server topics.
        # This prevents the topics from appearing as being streamed from
        # the server.
        self.stream_watchlist = []

        # Define the top layout (connection + stats)
        ly_top = QVBoxLayout()
        ly_top.addWidget(self.connection_widget)
        ly_top.addWidget(self.stats_widget)
        ly_top.setAlignment(Qt.AlignTop)

        # Define the top splitter (topics list + topic stats)
        self.server_splitter = QSplitter(self)
        self.server_splitter.splitterMoved.connect(
            partial(self.configure_panel, self.server_topics_panel_wg)
        )
        self.server_splitter.addWidget(self.server_topics_list_wg)
        self.server_splitter.addWidget(self.server_topics_panel_wg)

        # Define the bottom splitter
        self.client_splitter = QSplitter(self)
        self.client_splitter.splitterMoved.connect(
            partial(self.configure_panel, self.client_topics_panel_wg)
        )
        self.client_splitter.addWidget(self.client_topics_list_wg)
        self.client_splitter.addWidget(self.client_topics_panel_wg)

        # Create checkboxes
        self.server_cb = QCheckBox("Topics streaming from server")
        self.server_cb.setChecked(True)
        self.server_cb.stateChanged.connect(self.toggle_top)
        self.client_cb = QCheckBox("Topics streaming to server")
        self.client_cb.setChecked(False)
        self.client_cb.stateChanged.connect(self.toggle_bottom)

        # Create notice label
        lb_notice = QLabel("@2022, Kiwibot, Inc. or its Affiliates, Ai&Robotics")
        lb_notice.setAlignment(Qt.AlignCenter)

        # Define the main layout for window
        ly_main = QVBoxLayout()
        ly_main.addLayout(ly_top, stretch=1)
        ly_main.addWidget(self.server_cb)
        ly_main.addWidget(self.server_splitter, stretch=10)
        ly_main.addWidget(self.client_cb)
        ly_main.addWidget(self.client_splitter, stretch=10)
        ly_main.addWidget(lb_notice)

        # Configure the bottom splitter to be hidden on default
        self.client_splitter.hide()

        # Define the central widget and set the layout
        wg_main = QWidget(self)
        wg_main.setLayout(ly_main)
        self.setCentralWidget(wg_main)

        # Declare the variables to store stats
        self.cpu_usage = 0.0
        self.roundtrip = 0.0
        self.download_speed = 0.0
        self.old_bytes_recv = 0.0

        # Create timers to update each of the interface stats
        cpu_usage_timer = QTimer(self)
        roundtrip_timer = QTimer(self)
        download_speed_timer = QTimer(self)
        cpu_usage_timer.timeout.connect(self.update_cpu_usage)
        roundtrip_timer.timeout.connect(self.update_roundtrip)
        download_speed_timer.timeout.connect(self.update_download_speed)
        cpu_usage_timer.start(cpu_timer_to)
        roundtrip_timer.start(rtt_timer_to)
        download_speed_timer.start(download_timer_to)

        # Create a timer to update the stats panel
        stats_timer = QTimer(self)
        stats_timer.timeout.connect(self.update_stats)
        stats_timer.start(global_stats_timer_to)

        # Create a timer to update the topic stats
        self.topic_stats_timer = QTimer(self)
        self.topic_stats_timer.timeout.connect(self.update_topic_stats_and_state)
        self.topic_stats_to = topic_stats_timer_to

        # Timer that checks the connection status to handle closures
        self.restore_timer = QTimer(self)
        self.restore_timer.timeout.connect(self.restore_interface_on_reconnection)

        # Timer to update the available topics
        self.topic_upd_timer = QTimer(self)
        self.topic_upd_timer.timeout.connect(self.update_available_topics)
        self.topic_upd_timer_to = topic_upd_timer_to

    def closeEvent(self, event: PyQt5.QtGui.QCloseEvent):
        """! Function for handling the close interface event."""
        if self.client is not None:
            self.client.stop_reactor()
            if self.client.protocol.is_connected:
                self.disconnect_from_server()
        self.stop_threads = True
        super(QMainWindow, self).closeEvent(event)

    def resizeEvent(self, event: PyQt5.QtGui.QResizeEvent):
        """Update the user interface on resize."""
        self.configure_panel(self.server_topics_panel_wg)
        self.configure_panel(self.client_topics_panel_wg)
        super().resizeEvent(event)

    def toggle_top(self):
        """! Toggle the server (server to client) topic stream widget.

        This method evaluates the checkbox status of the server widget. If
        the checkbox is set, the server widget is shown. If the checkbox is
        not set, the server widget will be hidden.
        """
        if self.server_cb.isChecked():
            self.server_splitter.show()
            self.configure_panel(self.server_topics_panel_wg)
        else:
            self.server_splitter.hide()

    def toggle_bottom(self):
        """! Toggle the client (client to server) topic stream widget.

        This method evaluates the checkbox status of the client widget. If
        the checkbox is set, the client widget is shown. If the checkbox is
        not set, the client widget will be hidden.
        """
        if self.client_cb.isChecked():
            self.client_splitter.show()
            self.configure_panel(self.client_topics_panel_wg)
        else:
            self.client_splitter.hide()

    def configure_panel(self, panel):
        """! Configure a panels columns according to its size.

        This method configures the number of columns in a panel depending
        on its width. The number of columns configuration is done according
        to the following breakpoints:
         - (width < 500): single (1) column.
         - (500 < width < 700): two (2) columns.
         - (700 < width < 900): three (3) columns.
         - (900 < width < 1100): four (4) columns.
         - (width > 1100): five (5) columns.

        @param panel "" instance of a panel that will be configured depending
        on its size.
        """
        panel_width = panel.size().width()
        if panel_width < 501:
            panel.MAX_COLS = 1
            panel.configure_panel()
        if panel_width > 500 and panel_width < 701:
            panel.MAX_COLS = 2
            panel.configure_panel()
        if panel_width > 700 and panel_width < 901:
            panel.MAX_COLS = 3
            panel.configure_panel()
        if panel_width > 900 and panel_width < 1101:
            panel.MAX_COLS = 4
            panel.configure_panel()
        if panel_width > 1100 and panel_width < 1301:
            panel.MAX_COLS = 5
            panel.configure_panel()

    def reset_network_attributes(self):
        """! Reset the network-related attributes of the class."""
        self.client = None
        self.server_ip_addr = None
        self.is_connected = False
        self.retry_connection = False
        self.topic_handlers = {}
        self.available_topics = []

    def show_warning_message(self, title: str, message: str):
        """! Show a warning message box in the interface.
        @param title "str" title for the warning.
        @param message "str" content for the warning.
        """
        msg = QMessageBox(parent=self)
        msg.setIcon(QMessageBox.Warning)
        msg.setText(message)
        msg.setWindowTitle(title)
        msg.exec_()

    def restore_interface_on_reconnection(self):
        """! Check the connection status and restore the interface.

        This function test the connection status of the client and
        calls the 'restore_interface' function to restore the
        interface if a reconnection is found to take place.
        """
        if self.client is not None:
            self.is_connected = self.client.protocol.is_connected
            if self.retry_connection and self.is_connected:
                self.restore_interface()
                self.retry_connection = False
                self.connection_widget.set_status_label("CONNECTED")
            if self.retry_connection and not self.is_connected:
                self.connection_widget.set_status_label("RETRYING")
            else:
                self.retry_connection = not self.is_connected
        else:
            self.is_connected = False

    def restore_interface(self):
        """! Restore the interface after a reconnection."""
        self.node.get_logger().info("Restoring interface after reconnection.")

        # Get current topics to restore them later.
        server_streamed_topics = self.server_topics_panel_wg.get_current_topics()
        client_streamed_topics = self.client_topics_panel_wg.get_current_topics()

        # Delete the topic handlers.
        for topic in list(self.topic_handlers.keys()):
            self.topic_handlers[topic].destroy_subscription()
            del self.topic_handlers[topic]

        # Delete the topic streamers
        for streamer in self.streamers:
            self.streamers.remove(streamer)
            streamer.destroy_subscription()

        # Clean the interface
        self.server_topics_list_wg.clear_list()
        self.server_topics_panel_wg.remove_all_topics()
        self.client_topics_list_wg.clear_list()
        self.client_topics_panel_wg.remove_all_topics()

        # Configure local available topics
        local_available_topics = self.get_current_local_topics()
        server_available_topics = self.client.get_available_topics()

        # Configure list with available topics
        for topic in server_available_topics:
            self.server_topics_list_wg.add_topic(topic)
        for topic in local_available_topics:
            self.client_topics_list_wg.add_topic(topic)

        # Add topics to panels
        for topic in server_streamed_topics:
            self.add_topic_to_panel(topic)
        for topic in client_streamed_topics:
            self.add_client_to_panel(topic)

    def update_cpu_usage(self):
        """! Update the CPU usage statistic value."""
        self.cpu_usage = cpu_percent()

    def update_roundtrip(self):
        """! Update the roundtrip response time value."""
        if self.client is not None:
            if self.client.protocol.is_connected:
                try:
                    ping_response = ping(
                        address=self.server_ip_addr,
                        count=1,
                        timeout=0.5,
                        privileged=False,
                    )
                    self.roundtrip = ping_response.avg_rtt
                except gaierror as ge:
                    self.node.get_logger().error(
                        "Can not connect to server to get rtt!"
                    )
                except NameLookupError as nle:
                    self.node.get_logger().error(
                        "The server address is no longer available!"
                    )
                except Exception as ge:
                    self.node.get_logger().error(
                        f"There was an error while getting roundtrip: {ge}"
                    )
        else:
            self.roundtrip = 0.0

    def update_download_speed(self):
        """! Update the download speed statistic value."""
        net_if_stats_val = net_io_counters()
        received_bytes = net_if_stats_val.bytes_recv - self.old_bytes_recv
        self.old_bytes_recv = net_if_stats_val.bytes_recv
        self.download_speed = (received_bytes / 1024.0) / 0.25

    def update_stats(self):
        """! Update the statistics value in the user interface."""
        self.stats_widget.update_stats_widget(
            self.cpu_usage, self.roundtrip, self.download_speed
        )

    def update_available_topics(self):
        """! Updates the available topics in the interface.

        This function checks for the currently available topics from the
        server. If a topic is not in the interface but in the available
        topics, it is added to the topic list.

        If a topic is in the interface but not in the available topics, it is
        removed only if it is in the topic list. If topic is in the topics
        panel, it will not be removed.
        """
        # Check if the client is connected to the server
        if self.client.protocol.is_connected:

            # Get the server and client available topics
            server_current_topics = self.client.get_available_topics()
            client_current_topics = self.get_current_local_topics()

            # Get the topics streamed from server
            subscribed_topics = self.server_topics_panel_wg.get_current_topics()
            streamed_topics = self.client_topics_panel_wg.get_current_topics()

            # List to store available topics
            server_available_topics, client_available_topics = [], []

            # Server available topics
            server_available_topics = [
                topic
                for topic in server_current_topics
                if topic not in subscribed_topics
                and topic not in self.stream_watchlist
                and topic not in streamed_topics
            ]

            # Clear the watchlist
            self.stream_watchlist = []

            # Client available topics
            client_available_topics = [
                topic
                for topic in client_current_topics
                if topic not in streamed_topics and topic not in subscribed_topics
            ]

            # Get topics present in lists
            server_topics_list = self.server_topics_list_wg.get_current_topics()
            client_topics_list = self.client_topics_list_wg.get_current_topics()

            # Get the difference between lists
            server_add_topics = list(
                set(server_available_topics) - set(server_topics_list)
            )
            server_rm_topics = list(
                set(server_topics_list) - set(server_available_topics)
            )
            client_add_topics = list(
                set(client_available_topics) - set(client_topics_list)
            )
            client_rm_topics = list(
                set(client_topics_list) - set(client_available_topics)
            )

            # Add topics to lists
            for topic in server_add_topics:
                self.server_topics_list_wg.insert_topic(topic)

            for topic in client_add_topics:
                self.client_topics_list_wg.insert_topic(topic)

            # Remove topics from lists
            for topic in server_rm_topics:
                self.server_topics_list_wg.remove_topic(topic)

            for topic in client_rm_topics:
                self.client_topics_list_wg.remove_topic(topic)

    def process_connection_address(self, address: str) -> Tuple[str, int]:
        """! Process the input address to define a server host/port pair.

        @param address "str" input address that will be processed.

        @return "str" host address for connection.
        @return "int" port for connection.
        """
        match = re.search(RosboardClientGui.URL_RE, address)
        scheme = (
            match.group("scheme")
            if match.group("scheme") is not None
            and match.group("scheme") in RosboardClientGui.PORTS.keys()
            else "DEFAULT"
        )
        host = match.group("host")
        port = (
            match.group("port")
            if match.group("port") is not None
            else RosboardClientGui.PORTS[scheme]
        )
        print(f"Connecting to: {host}:{port}")
        return host, port

    def test_connection(self, host: str, port: int) -> bool:
        """! Check the websocket status and enable the connect button."""
        try:
            test_socket = socket(AF_INET, SOCK_STREAM)
            test_socket.settimeout(1.0)
            is_avail = test_socket.connect_ex((host, int(port))) == 0
            test_socket.close()
            if not is_avail:
                self.show_warning_message(
                    "Can not connect to server!", "Connection error!"
                )
            return is_avail
        except ValueError as e:
            self.show_warning_message(
                "Target web socket is not enabled. Is the port correct?",
                "Target web socket is not enabled!",
            )
            return False
        except gaierror as e:
            self.show_warning_message(
                "Target web socket is not enabled!",
                "Target web socket is not enabled. Is the address correct?",
            )
            return False
        except Exception as ge:
            self.node.get_logger().error(
                f"There was an error while trying to test the connection to server: {ge}"
            )

    def connect_to_server(self):
        """! Function to connect to rosboard server.

        This function will attempt to connect to the rosboard server. In order
        to do so, it will first test the connection based on the parameters
        that were input in the connection address field. Then, it will create
        a RosboardClient instance and configure the interface according to the
        current available topics.
        """
        # Get the connection parameters from the connection widget.
        address = self.connection_widget.get_connection_address()
        host, port = self.process_connection_address(address)

        # Test the connection before connecting.
        if self.test_connection(host, port):
            try:
                # Connect to the rosboard client
                self.client = RosboardClient(
                    host=f"{host}:{port}", connection_timeout=5.0
                )

                # Get topics list and add them to the topics list widget.
                self.topic_handlers = {}
                self.available_topics = self.client.get_available_topics()
                for topic in self.available_topics:
                    self.server_topics_list_wg.add_topic(topic)

                # Get topics from client and add them to the client topics list
                self.client_topics = self.get_current_local_topics()

                for topic_name in self.client_topics:
                    self.client_topics_list_wg.add_topic(topic_name)

                # Start the timers to update topics list, stats and restore interface
                self.topic_stats_timer.start(250)
                self.topic_upd_timer.start(5000)
                self.restore_timer.start(500)

                # Store the connection address and set flag
                self.server_ip_addr = host
                self.is_connected = True

                # Configure the connection widget to connected status
                self.connection_widget.set_buttons_status(self.is_connected)
                self.connection_widget.toggle_edits(False)
                self.connection_widget.set_status_label("CONNECTED")

            except Exception as e:
                self.reset_network_attributes()
                self.node.get_logger().error(str(e))
                self.show_warning_message(
                    "Timeout while connecting",
                    "There was a timeout when trying to connect to server.",
                )

    def disconnect_from_server(self):
        """! Disconnects the interface from the rosboard server.

        This function will destroy the socket connections to topics in the
        server, destroy the connection and restore the interface to its
        disconnected status.
        """
        # Stop timers to update topics list, stats and interface restore.
        self.topic_upd_timer.stop()
        self.topic_stats_timer.stop()
        self.restore_timer.stop()

        # Destroy the socket connections for each topic
        for topic in list(self.topic_handlers.keys()):
            self.topic_handlers[topic].destroy_subscription()
            del self.topic_handlers[topic]

        # Reset the interface network attributes
        self.reset_network_attributes()

        # Restore the interface to the disconnected status
        self.server_topics_list_wg.clear_list()
        self.server_topics_panel_wg.remove_all_topics()
        self.client_topics_list_wg.clear_list()
        self.client_topics_panel_wg.remove_all_topics()
        self.connection_widget.toggle_edits(True)
        self.connection_widget.set_buttons_status(self.is_connected)
        self.connection_widget.set_status_label("DISCONNECTED")

    def get_current_local_topics(self) -> list:
        """! Function to get available topics from client side.
        @return "list" topic names available in the client.
        """
        current_topics = []
        for topic in self.node.get_topic_names_and_types():
            if len(self.node.get_publishers_info_by_topic(topic[0])) > 0:
                current_topics.append(topic[0])
        return current_topics

    def add_topic_to_panel(self, topic_name: str):
        """! Add a topic to the topics panel widget and create its handler.

        Calling this method will create an instance of 'TopicHandler', which
        will handle the websocket/ROS interface of the topics, and will add
        the topic to the topics panel widget.

        @param topic_name "str" name of the topic that will be added to panel.
        """
        try:
            self.topic_handlers[topic_name] = TopicHandler(
                topic_name, self.client, self.node
            )
            self.server_topics_list_wg.remove_topic(topic_name)
            self.server_topics_panel_wg.add_topic(topic_name)
        except ModuleNotFoundError as e:
            self.show_warning_message(
                "Can not load message!", "Can not load topic message type!"
            )
        except Exception as e:
            self.server_topics_list_wg.remove_topic(topic_name)
            self.node.get_logger().warning(
                "Attempted to subscribe to unavailable topic!"
            )

    def add_client_to_panel(self, topic_name: str):
        """! Create a topic streamer and add it to panel.
        @param topic_name "str" name of the topic that will be streamed.
        """
        self.streamers.append(GenericStreamer(self.node, self.client, topic_name))
        self.client_topics_panel_wg.add_topic(topic_name)
        self.client_topics_list_wg.remove_topic(topic_name)

    def move_from_client_panel_to_list(self, topic_name: str):
        """! Stop streaming topic and add it to list.
        @param topic_name "str" name of the topic that will stop streaming.
        """
        for current_index in range(len(self.streamers)):
            if self.streamers[current_index].topic_name == topic_name:
                topic_streamer = self.streamers.pop(current_index)
                topic_streamer.destroy_subscription()
                break
        self.client_topics_panel_wg.remove_topic(topic_name)
        self.client_topics_list_wg.insert_topic(topic_name)

        # Append the removed topic to watchlist
        self.stream_watchlist.append(topic_name)

    def move_from_server_panel_to_list(self, topic_name: str):
        """! Add a topic to list widget and remove it from panel if necessary

        Calling this method will add a given topic to the topics list in
        TopicsListWidget. If the topic is being handled (is present in the)
        topics panel, such handling will be terminated.

        @param topic_name "str" topic that will be added to the topics list.
        """
        if topic_name in self.topic_handlers.keys():
            self.topic_handlers[topic_name].destroy_subscription()
            del self.topic_handlers[topic_name]
            self.server_topics_panel_wg.remove_topic(topic_name)
        self.server_topics_list_wg.insert_topic(topic_name)

    def update_topic_stats_and_state(self):
        """! Update the handled topic statistics and state."""
        topic_stats = {}
        topic_state = {}
        for topic in self.topic_handlers.keys():
            topic_stats[topic] = self.topic_handlers[topic].get_topic_stats()
            topic_state[topic] = self.topic_handlers[topic].state
        self.server_topics_panel_wg.update_topic_stats(topic_stats)
        self.server_topics_panel_wg.update_topic_state(topic_state)


class ConnectionWidget(QWidget):

    # Dictionary to update the connection status label.
    CONN_STATUS_DICT = {
        "RETRYING": "QLabel#StatusLabel{background-color: #FFE666;}",
        "CONNECTED": "QLabel#StatusLabel{background-color: #77CC66;}",
        "DISCONNECTED": "QLabel#StatusLabel{background-color: #FF6666;}",
    }

    def __init__(self, parent: RosboardClientGui):
        """! Widget to establish a connection to the server.

        This class has an input line in which the server URL can be typed by
        the user. Two buttons allow for connecting and disconnecting the user
        interface to/from the server. Finally, a label is used to show the
        connection status.

        @param parent "RosboardClientGui" establish the parent class of the
        widget.
        """
        super(QWidget, self).__init__(parent)
        self.setObjectName("ConnectionWidget")

        # Create the line edits for the IP Address and the port
        self.address_le = QLineEdit()
        self.address_le.setText("localhost:8888")

        # Create the buttons for connecting and disconnecting the buttons
        self.connect_bt = QPushButton("CONNECT")
        self.connect_bt.setEnabled(True)
        self.disconnect_bt = QPushButton("DISCONNECT")
        self.disconnect_bt.setEnabled(False)
        self.connect_bt.clicked.connect(self.parent().connect_to_server)
        self.disconnect_bt.clicked.connect(self.parent().disconnect_from_server)

        # Label to provide feedback on connection status
        self.status_lb = QLabel("DISCONNECTED")
        self.status_lb.setObjectName("StatusLabel")
        self.setStyleSheet("QLabel#StatusLabel{background: red;}")

        # Define the widget layout
        ly_widget = QGridLayout()
        ly_widget.setSpacing(0)
        ly_widget.setContentsMargins(0, 0, 0, 0)
        ly_widget.addWidget(self.address_le, 0, 0, 1, 2)
        ly_widget.addWidget(self.connect_bt, 1, 0)
        ly_widget.addWidget(self.disconnect_bt, 1, 1)
        ly_widget.addWidget(self.status_lb, 2, 0, 1, 2, Qt.AlignCenter)
        self.setLayout(ly_widget)

    def get_connection_address(self) -> str:
        """! Get the current text value of the connection address.
        @return "str" value of the line edit.
        """
        return self.address_le.text()

    def set_buttons_status(self, is_connected: bool):
        """! Enables or disables the buttons depending on connection status.

        The connect button will be enabled if the interface is not connected
        to the server. It will be disabled otherwise. The disconnect button
        will be enabled if the interface is connected to the server. It will
        be disabled otherwise.

        @param is_connected "bool" indicate if the GUI is connected or not.
        """
        self.connect_bt.setEnabled(not is_connected)
        self.disconnect_bt.setEnabled(is_connected)

    def toggle_edits(self, enabled: bool):
        """! Toggle the widget line edits to allow for changes or not.
        @param enabled "bool" indicates if the line edits are enabled or not.
        """
        self.address_le.setEnabled(enabled)

    def set_status_label(self, status: str):
        """! Set the status label display based on parameters.
        @param status "str" connection status to server.
        """
        self.status_lb.setText(status)
        self.setStyleSheet(ConnectionWidget.CONN_STATUS_DICT[status])


class StatusWidget(QWidget):
    def __init__(self, parent: RosboardClientGui):
        """! Widget to present global statistics in the user interface.

        Widget that contains elements to show the stats for the user.
        The stats consist of the CPU usage represented as a percentage,
        the round trip time to the connected socket, and the current
        download speed.

        @param parent "RosboardClientGui" establish the parent class of the
        widget.
        """
        super(QWidget, self).__init__(parent)

        # Define the labels to store the stats. values.
        self.cpu_usage_lb = QLabel("XXX.XX %")
        self.roundtrip_lb = QLabel("XXX.XX ms")
        self.download_lb = QLabel("XXXX.X kbps")

        # Define the widget layout
        ly_widget = QGridLayout()
        ly_widget.addWidget(QLabel("CPU USAGE [%]"), 0, 0, Qt.AlignCenter)
        ly_widget.addWidget(QLabel("ROUNDTRIP [ms]"), 0, 1, Qt.AlignCenter)
        ly_widget.addWidget(QLabel("DOWNLOAD SPEED [kbps]"), 0, 2, Qt.AlignCenter)
        ly_widget.addWidget(self.cpu_usage_lb, 1, 0, Qt.AlignCenter)
        ly_widget.addWidget(self.roundtrip_lb, 1, 1, Qt.AlignCenter)
        ly_widget.addWidget(self.download_lb, 1, 2, Qt.AlignCenter)
        self.setLayout(ly_widget)

    def update_stats_widget(self, cpu_usage: float, roundtrip: float, download: float):
        """!
        Update the statistic widget with the CPU usage, roundtrip and download speed.
        @param cpu_usage "float" value that represent the CPU usage.
        @param roundtrip "float" value that represents the network delay.
        @param download "float" value that represents the download speed.
        """
        self.cpu_usage_lb.setText(f"{cpu_usage:3.2f} %")
        self.roundtrip_lb.setText(f"{roundtrip:3.2f} ms")
        self.download_lb.setText(f"{download:4.1f} kbps")


class TopicsListWidget(QWidget):
    def __init__(self, parent: RosboardClientGui, button_click_handler: callable):
        """! Widget to show a topics list and manage its behavior.

        This class includes the logic to handle a topics list. The logic
        allows to add and remove buttons. Buttons can be added at the end of
        the list ('add_topic') or inserted to match an alphabetical order
        ('insert_topic'). Removing a specific button ('') is possible as
        clearing the whole list (). When a button is pressed, the widget will
        route the slot into a callable object that is passed in the
        constructor.

        @params parent "RosboardClientGui" set the parent class of the
        widget to call methods.
        @params button_click_handler "callable" get the callable function that
        is executed when a button from the list is pressed. The callable must
        have a parameter related to the topic name.
        """
        super(TopicsListWidget, self).__init__(parent)
        self.setObjectName("TopicsListWidget")
        self.setMinimumWidth(300)

        # Store the callable object
        self.button_slot = button_click_handler

        # List to store the button objects
        self.buttons_list = []

        # Create the layout to add the buttons
        self.ly_buttons = QVBoxLayout()
        self.ly_buttons.setAlignment(Qt.AlignTop)

        # Create the group box to handle the buttons layout
        gb_buttons = QGroupBox()
        gb_buttons.setObjectName("topics_gb")
        gb_buttons.setLayout(self.ly_buttons)
        gb_buttons.setStyleSheet("QGroupBox#topics_gb{border: 1px solid black;}")

        # Create the scroll area for the buttons
        sa_buttons = QScrollArea()
        sa_buttons.setWidget(gb_buttons)
        sa_buttons.setWidgetResizable(True)

        # Create the main layout to add the scroll area
        ly_main = QVBoxLayout()
        ly_main.addWidget(sa_buttons)
        self.setLayout(ly_main)

    def add_topic(self, topic_name):
        """! Add a button to the at the end of the topic list.
        @param topic_name "str" name of the topic that will be linked to the button.
        """
        bt_topic = QPushButton(topic_name)
        bt_topic.clicked.connect(partial(self.button_slot, topic_name))
        self.buttons_list.append(bt_topic)
        self.ly_buttons.addWidget(bt_topic)

    def insert_topic(self, topic_name):
        """! Insert a topic to the list in alphabetic order.
        @param topic_name "str" name of the topic that will be linked to the button.
        """
        # Create the button
        button = QPushButton(topic_name)
        button.clicked.connect(partial(self.button_slot, topic_name))

        # Get the insertion point in list
        ins_point = bisect.bisect_left(self.get_current_topics(), topic_name)
        self.buttons_list.insert(ins_point, button)
        self.ly_buttons.insertWidget(ins_point, button)

    def remove_topic(self, topic_name):
        """! Remove a topic button from the list.
        @param topic_name "str" name of the topic whose button will be removed.
        """
        for button in self.buttons_list:
            if button.text() == topic_name:
                self.buttons_list.remove(button)
                button.deleteLater()
                break

    def clear_list(self):
        """! Remove every button present in the list."""
        for button in self.buttons_list:
            button.deleteLater()
        self.buttons_list = []

    def get_current_topics(self) -> list:
        """! Return the current topic names in the widget."""
        return [bt.text() for bt in self.buttons_list]


class TopicWidget(QWidget):
    def __init__(self, parent, topic_name: str, close_slot: callable):
        """!"""
        super(QWidget, self).__init__(parent)
        self.setObjectName("TopicWidget")
        self.setAttribute(Qt.WA_StyledBackground)

        # Store the topic name as class attribute
        self.topic_name = topic_name

        # Add the close button and connect its slot to parameter
        bt_close = QPushButton("x")
        bt_close.clicked.connect(partial(close_slot, self.topic_name))

        lb_name = QLabel(self.topic_name)
        lb_name.setObjectName("TopicName")

        ly_top = QHBoxLayout()
        ly_top.setSpacing(5)
        ly_top.setContentsMargins(0, 0, 0, 0)
        ly_top.addWidget(lb_name, stretch=100)
        ly_top.addWidget(bt_close, stretch=1, alignment=Qt.AlignRight)

        self.ly_widget = QGridLayout()
        self.ly_widget.setSpacing(0)
        self.ly_widget.addLayout(ly_top, 0, 0, 1, 2)
        self.setLayout(self.ly_widget)


class ExtendedTopicWidget(TopicWidget):

    # Dictionary that maps the topic state to a widget background color.
    TOPIC_STATE_DICT = {
        "DELAY": "QWidget#TopicWidget{background-color: #FF6666;}",
        "NORMAL": "QWidget#TopicWidget{background-color: #77CC66;}",
        "NO_DATA": "QWidget#TopicWidget{background-color: #B0B0B0;}",
    }

    def __init__(self, parent, topic_name: str, close_slot: callable):
        """!"""
        super(ExtendedTopicWidget, self).__init__(parent, topic_name, close_slot)

        self.freq_lb = QLabel("XXX.XX")
        self.latency_lb = QLabel("XXX.XX")

        self.ly_widget.addWidget(QLabel("Freq. [Hz]:"), 1, 0)
        self.ly_widget.addWidget(QLabel("DT [ms]:"), 2, 0)
        self.ly_widget.addWidget(self.freq_lb, 1, 1, Qt.AlignRight)
        self.ly_widget.addWidget(self.latency_lb, 2, 1, Qt.AlignRight)

    def update_topic_stats(self, frequency: float, latency: float):
        """! Update the topic statistics: frequency and latency.
        @param frequency "float" value.
        @param latency "float" value. Might be 'None' to indicate latency is not present.
        latency calculations or not.
        """
        has_latency = latency is not None
        self.freq_lb.setText(f"{frequency:3.2f}")
        if has_latency:
            self.latency_lb.setText(f"{latency * 1000:3.2f}")
        else:
            self.latency_lb.setText("N/A")

    def update_topic_state(self, state: str):
        """!
        Update the node element color depending on topic state.
        @param state "str" represent the received messages values for topic.
        """
        self.setStyleSheet(ExtendedTopicWidget.TOPIC_STATE_DICT[state])


class TopicsPanelWidget(QWidget):
    def __init__(self, parent: RosboardClientGui, widget, close_slot: callable):
        """! Widget that contains handled topic widgets.

        This widget contains the topics widgets associated to the handled topics.
        Handled topics are the ones that are being published in the client side.
        Elements in this widget consists of further widgets that include the topics
        name and statistics. This widget is only responsible of organizing how such
        widgets are presented.

        @param parent "RosboardClientGui" establish the parent class of the
        widget.
        @param widget "QWidget" widget that will be added to the panel. The
        widget must have two parameters fields: a string related to the
        topics name and a callable that is called when the topic is closed.
        @Param close_slot "callable" function that is called when the widget
        is closed.
        """
        super(QWidget, self).__init__(parent)
        self.setObjectName("TopicsPanelWidget")
        self.setMinimumWidth(300)

        # Store the passed parameters
        self.widget_class = widget
        self.close_slot = close_slot

        # Define attributes for max. columns
        self.MAX_COLS = 4

        # Define the layout for the widget
        self.ly_widget = QGridLayout()
        self.ly_widget.setAlignment(Qt.AlignTop)

        # Create the scroll area
        topics_gb = QGroupBox()
        topics_gb.setObjectName("topics_gb")
        topics_gb.setLayout(self.ly_widget)
        topics_gb.setStyleSheet("QGroupBox#topics_gb{border: 1px solid black;}")

        scroll_area = QScrollArea()
        scroll_area.setWidget(topics_gb)
        scroll_area.setWidgetResizable(True)

        ly_main = QVBoxLayout()
        ly_main.addWidget(scroll_area)
        self.setLayout(ly_main)

        # List with topic widgets
        self.widgets_list = []

    def add_topic(self, topic_name: str):
        """! Add topic to panel and configure the layout.
        @param topic_name "str" name of the topic that will be added.
        """
        topic_widget = self.widget_class(
            self, topic_name=topic_name, close_slot=self.close_slot
        )
        self.widgets_list.append(topic_widget)
        self.configure_panel()

    def remove_topic(self, topic_name: str):
        """! Remove a topic from the panel and configure the layout.
        @param topic_name "str" name of the topic that will be removed.
        """
        for topic_wg in self.widgets_list:
            if topic_wg.topic_name == topic_name:
                self.widgets_list.remove(topic_wg)
                topic_wg.deleteLater()
        self.configure_panel()

    def remove_all_topics(self):
        """!
        Removes every topic from the panel.
        """
        topic_names = [topic_wg.topic_name for topic_wg in self.widgets_list]
        for tn in topic_names:
            self.remove_topic(tn)

    def update_topic_stats(self, topic_stats: dict):
        """! Update the statistics for topics in panel.
        @param topic_stats "dict" dictionary with the topic stats. Dictionary
        key is topic name and value is a list with topic statistics.
        """
        for widget in self.widgets_list:
            stats = topic_stats[widget.topic_name]
            widget.update_topic_stats(stats[0], stats[1])

    def update_topic_state(self, topic_state: dict):
        """! Update the topic state on the each handled widget.
        @param topic_state "dict" dictionary with the state for each topic.
        """
        for widget in self.widgets_list:
            state = topic_state[widget.topic_name]
            widget.update_topic_state(state)

    def configure_panel(self):
        """! Adds and organizes the topics widgets."""
        count = 0
        for widget in self.widgets_list:
            self.ly_widget.addWidget(
                widget, count // self.MAX_COLS, count % self.MAX_COLS
            )
            count += 1

    def get_current_topics(self) -> list:
        """! Return the current topics in widget."""
        return [topic_wg.topic_name for topic_wg in self.widgets_list]


def main():
    rclpy.init(args=sys.argv)

    app = QApplication(sys.argv)
    app.setApplicationName("Rosboard Client GUI")
    ui = RosboardClientGui()

    # Spin node
    executor = MultiThreadedExecutor()
    spin_thread = Thread(target=rclpy.spin, args=(ui.node, executor))
    spin_thread.daemon = True
    spin_thread.start()

    ui.showMaximized()
    sys.exit(app.exec())
