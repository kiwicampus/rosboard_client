#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
"""
Code Information:
    Code Information: PyQt5 user interface for the rosboard client.
    Maintainer: Nicolas Rocha Pacheco
	Mail: nicolas.rocha@kiwibot.com
"""

# =============================================================================
import os
import re
import sys
from typing import Tuple

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_prefix

from icmplib import ping, NameLookupError
from time import time, sleep
from threading import Thread
from psutil import cpu_percent, net_io_counters
from socket import socket, AF_INET, SOCK_STREAM, gaierror

from functools import partial
import PyQt5
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QWidget,
    QLabel,
    QHBoxLayout,
    QVBoxLayout,
    QGridLayout,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QGroupBox,
    QMessageBox,
    QSplitter,
)

from rosboard_desktop_client.networking import RosboardClient
from rosboard_desktop_client.republishers import PublisherManager


class TopicHandler:

    AVG_SAMPLES = 20
    ALPHA = 2.0 / (AVG_SAMPLES + 1.0)
    C_ALPHA = 1 - ALPHA

    def __init__(self, topic_name: str, client: RosboardClient, node: Node):
        """! Class to handle the topic subscription and statistics.

        This class provides a mechanism to connect a topic to the rosboard
        client and to generate statistics measurements for topics such
        as the current latency and rate in which messages are received.

        @param topic_name "str" name of the topic being handled.
        @param client "RosboardClient" client for the topic.
        @param node "Node" ROS node instance to communicate with ROS.
        """
        # Initialize attributes
        self.node = node
        self.client = client
        self.topic_name = topic_name
        self.has_header = False
        self.rate = 0.0
        self.avg_rate = 0.0
        self.latency = 0.0
        self.latency_list = []
        self.rate_list = []
        self.state = "NO_DATA"

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
            if self.t_last_msg is not None:
                if time() - self.t_last_msg > 5.0:
                    self.state = "NO_DATA"
                elif self.rate < 0.8 * self.avg_rate:
                    self.state = "DELAY"
                else:
                    self.state = "NORMAL"
            sleep(0.25)

    def calculate_average(self, time_list: list) -> float:
        """! Calculate the exponentially weighted moving average.

        This function calculates the exponentially weighted moving average
        (EWMA) for a given list of values. The function does not expect the
        list to have a fixed value, although it will use the predefined
        'ALPHA' constant, which might depend on the expected number of items
        of the list.

        @param time_list "list" contains the latest time values between
        messages.
        @return "float" with the average value. Return 0.0 if time_list has
        no values.
        """
        average = 0.0
        list_size = len(time_list)
        if list_size > 0:
            average = time_list[0]
            if list_size > 1:
                for indx in range(1, list_size):
                    average = (
                        TopicHandler.ALPHA * time_list[indx]
                        + TopicHandler.C_ALPHA * average
                    )
        return average

    def topic_callback(self, msg: list):
        """! Topic callback function for incoming rosboard messages.

        This function processes the incoming messages in order to provide the
        stats of the
        """
        self.n_msgs += 1
        t_current = time()
        if self.t_last_msg is not None:
            if self.has_header:
                t_send = self.timestamp_to_secs(msg[1]["header"]["stamp"])
                self.latency_list.append(t_current - t_send)
                self.latency_list = self.latency_list[-TopicHandler.AVG_SAMPLES :]

            self.rate_list.append(t_current - self.t_last_msg)
            self.rate_list = self.rate_list[-TopicHandler.AVG_SAMPLES :]

        else:
            self.has_header = "header" in msg[1].keys()

        # Always execute this code block
        self.t_last_msg = t_current
        self.republisher.parse_and_publish(msg)

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
        t_average = self.calculate_average(self.rate_list)
        self.rate = 1.0 / t_average if t_average != 0.0 else 0.0
        self.latency = (
            self.calculate_average(self.latency_list) if self.has_header else None
        )
        return [self.rate, self.latency]


class RosboardClientGui(QMainWindow):

    URL_RE = "^((?P<scheme>[^:/?#]+):(?=//))?(//)?(((?P<login>[^:]+)(?::(?P<password>[^@]+)?)?@)?(?P<host>[^@/?#:]*)(?::(?P<port>\d+)?)?)?(?P<path>[^?#]*)(\?(?P<query>[^#]*))?"

    # List to store the valid protocols
    valid_protocols = ["ws", "wss", "http", "https", "tcp"]

    def __init__(self):
        super(QMainWindow, self).__init__()
        self.setMinimumSize(650, 400)

        # Load stylesheet
        stylesheet_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "resources", "rosboard.qss"
        )
        with open(stylesheet_path, "r") as ss_file:
            self.setStyleSheet(ss_file.read())

        # Start the ROS node
        self.node = Node("rosboard_desktop_gui")

        self.reset_network_attributes()

        # Main window configurations
        self.setWindowTitle("Rosboard Client GUI")

        # Initialize custom widgets
        self.connection_widget = ConnectionWidget(self)
        self.stats_widget = StatsWidget(self)
        self.topics_list_widget = TopicsListWidget(self)
        self.topics_panel_widget = TopicsPanelWidget(self)

        # Define the top layout (connection + stats)
        ly_top = QHBoxLayout()
        ly_top.addWidget(self.connection_widget, 7)
        ly_top.addWidget(self.stats_widget, 3)

        # Define the bottom layout (topics list + topic stats)
        splitter_bottom = QSplitter(self)
        splitter_bottom.splitterMoved.connect(self.configure_topics_panel)
        splitter_bottom.addWidget(self.topics_list_widget)
        splitter_bottom.addWidget(self.topics_panel_widget)

        # Define the main layout for window
        ly_main = QVBoxLayout()
        ly_main.addLayout(ly_top, stretch=1)
        ly_main.addWidget(splitter_bottom, stretch=10)

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
        cpu_usage_timer.start(250)
        roundtrip_timer.start(1000)
        download_speed_timer.start(250)

        # Create a timer to update the stats panel
        stats_timer = QTimer(self)
        stats_timer.timeout.connect(self.update_stats)
        stats_timer.start(250)

        # Create a timer to update the topic stats
        self.topic_stats_timer = QTimer(self)
        self.topic_stats_timer.timeout.connect(self.update_topic_stats_and_state)

        # Timer that checks the connection status to handle closures
        self.restore_timer = QTimer(self)
        self.restore_timer.timeout.connect(self.restore_interface_on_reconnection)

        # Timer to update the available topics
        self.topic_upd_timer = QTimer(self)
        self.topic_upd_timer.timeout.connect(self.update_available_topics)

    def closeEvent(self, event: PyQt5.QtGui.QCloseEvent):
        """! Function for handling the close interface."""
        if self.client is not None:
            self.client.stop_reactor()
            if self.client.protocol.is_connected:
                self.disconnect_from_server()
        super(QMainWindow, self).closeEvent(event)

    def resizeEvent(self, event: PyQt5.QtGui.QResizeEvent):
        """Update the user interface on resize."""
        self.configure_topics_panel()
        super().resizeEvent(event)

    def configure_topics_panel(self):
        ui_width = self.topics_panel_widget.size().width()
        if ui_width < 501:
            self.topics_panel_widget.MAX_COLS = 1
            self.topics_panel_widget.configure_panel()
        if ui_width > 500 and ui_width < 701:
            self.topics_panel_widget.MAX_COLS = 2
            self.topics_panel_widget.configure_panel()
        if ui_width > 700 and ui_width < 901:
            self.topics_panel_widget.MAX_COLS = 3
            self.topics_panel_widget.configure_panel()
        if ui_width > 900 and ui_width < 1101:
            self.topics_panel_widget.MAX_COLS = 4
            self.topics_panel_widget.configure_panel()
        if ui_width > 1100 and ui_width < 1301:
            self.topics_panel_widget.MAX_COLS = 5
            self.topics_panel_widget.configure_panel()

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
        current_topics = self.topics_panel_widget.get_current_topics()

        # Delete the topic handlers.
        for topic in list(self.topic_handlers.keys()):
            self.topic_handlers[topic].destroy_subscription()
            del self.topic_handlers[topic]

        # Clean the interface
        self.topics_list_widget.remove_all_topics()
        self.topics_panel_widget.remove_all_topics()

        # Configure the interface
        available_topics = self.client.get_available_topics()
        for topic in available_topics:
            self.topics_list_widget.add_topic(topic)
        for topic in current_topics:
            self.add_topic_to_panel(topic)

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
                    pass
                except NameLookupError as nle:
                    pass
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

            # Get the currently available topics
            current_topics = self.client.get_available_topics()

            # Remove topics from topic list
            list_topics = self.topics_list_widget.get_current_topics()
            for topic in self.available_topics:
                if topic not in current_topics:
                    if topic in list_topics:
                        self.topics_list_widget.remove_topic(topic)
                    self.available_topics.remove(topic)

            # Add new topics to interface
            for topic in current_topics:
                if topic not in self.available_topics:
                    self.available_topics.append(topic)
                    self.add_topic_to_list(topic)

    def process_connection_address(self, address: str) -> Tuple[str, int]:
        """! Process the input address to define a server host/port pair.

        @param address "str" input address that will be processed.

        @return "str" host address for connection.
        @return "int" port for connection.
        """
        match = re.search(RosboardClientGui.URL_RE, address)
        host = match.group("host")
        port = match.group("port") if match.group("port") is not None else 80
        return host, port

    def test_connection(self, host: str, port: int) -> bool:
        """!
        Check the websocket status and enable the connect button.
        """
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
                    self.topics_list_widget.add_topic_at_end(topic)

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
        self.topics_list_widget.remove_all_topics()
        self.topics_panel_widget.remove_all_topics()
        self.connection_widget.toggle_edits(True)
        self.connection_widget.set_buttons_status(self.is_connected)
        self.connection_widget.set_status_label("DISCONNECTED")

    def add_topic_to_panel(self, topic_name: str):
        try:
            self.topic_handlers[topic_name] = TopicHandler(
                topic_name, self.client, self.node
            )
            self.topics_list_widget.remove_topic(topic_name)
            self.topics_panel_widget.add_topic(topic_name)
        except ModuleNotFoundError as e:
            self.show_warning_message(
                "Can not load message!", "Can not load topic message type!"
            )
        except Exception as e:
            self.node.get_logger().warning(
                "Attempted to subscribe to unavailable topic!"
            )

    def add_topic_to_list(self, topic_name: str):
        if topic_name in self.topic_handlers.keys():
            self.topic_handlers[topic_name].destroy_subscription()
            del self.topic_handlers[topic_name]
            self.topics_panel_widget.remove_topic(topic_name)
        self.topics_list_widget.add_topic(topic_name)

    def update_topic_stats_and_state(self):
        topic_stats = {}
        topic_state = {}
        for topic in self.topic_handlers.keys():
            topic_stats[topic] = self.topic_handlers[topic].get_topic_stats()
            topic_state[topic] = self.topic_handlers[topic].state
        self.topics_panel_widget.update_topic_stats(topic_stats)
        self.topics_panel_widget.update_topic_state(topic_state)


class ConnectionWidget(QWidget):
    """!
    Widget that contains the required elements to connect to websocket.
    """

    CONN_STATUS_DICT = {
        "RETRYING": "QLabel#StatusLabel{background-color: #FFE666;}",
        "CONNECTED": "QLabel#StatusLabel{background-color: #77CC66;}",
        "DISCONNECTED": "QLabel#StatusLabel{background-color: #FF6666;}",
    }

    def __init__(self, parent: RosboardClientGui):
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
        ly_widget.addWidget(QLabel("ADDRESS:"), 0, 0, 1, 2, Qt.AlignCenter)
        ly_widget.addWidget(self.address_le, 1, 0, 1, 2)
        ly_widget.addWidget(self.connect_bt, 2, 0)
        ly_widget.addWidget(self.disconnect_bt, 2, 1)
        ly_widget.addWidget(self.status_lb, 3, 0, 1, 2, Qt.AlignCenter)

        self.setLayout(ly_widget)

    def get_connection_address(self) -> str:
        return self.address_le.text()

    def set_buttons_status(self, is_connected: bool):
        """!
        Enables or disables the connection and disconnection buttons.
        @param is_connected "bool" indicate if the GUI is connected or not.
        """
        self.connect_bt.setEnabled(not is_connected)
        self.disconnect_bt.setEnabled(is_connected)

    def toggle_edits(self, enabled: bool):
        """!
        Toggle the widget line edits to allow for changes or not.
        @param enabled "bool" indicates if the line edits are enabled or not.
        """
        self.address_le.setEnabled(enabled)

    def set_status_label(self, status: str):
        """! Set the status label display based on parameters.

        @param status "str" connection status to server.
        """
        self.status_lb.setText(status)
        self.setStyleSheet(ConnectionWidget.CONN_STATUS_DICT[status])


class StatsWidget(QWidget):
    """!
    Widget that contains elements to show the stats for the user.
    The stats consist of the CPU usage represented as a percentage,
    the round trip time to the connected socket, and the current
    download speed.
    """

    def __init__(self, parent: RosboardClientGui):
        print(type(parent))
        super(QWidget, self).__init__(parent)

        # Define the labels to store the stats. values.
        self.cpu_usage_lb = QLabel("CPU USAGE: XXX.XX \t \t %")
        self.roundtrip_lb = QLabel("ROUNDTRIP: XXX.XX \t \t ms")
        self.download_lb = QLabel("DOWNLOAD: XXX.XX \t \t kbps")

        # Define the widget layout
        ly_widget = QGridLayout()
        ly_widget.addWidget(self.cpu_usage_lb, 0, 0)
        ly_widget.addWidget(self.roundtrip_lb, 1, 0)
        ly_widget.addWidget(self.download_lb, 2, 0)
        self.setLayout(ly_widget)

    def update_stats_widget(self, cpu_usage: float, roundtrip: float, download: float):
        """!
        Update the statistic widget with the CPU usage, roundtrip and download speed.
        @param cpu_usage "float" value that represent the CPU usage.
        @param roundtrip "float" value that represents the network delay.
        @param download "float" value that represents the download speed.
        """
        self.cpu_usage_lb.setText(f"CPU USAGE: {cpu_usage:3.2f} %")
        self.roundtrip_lb.setText(f"ROUNDTRIP: {roundtrip:3.2f} ms")
        self.download_lb.setText(f"DOWNLOAD: {download:4.1f} kbps")


class TopicsListWidget(QWidget):
    """!
    Widget that contains the required elements to connect to websocket.
    """

    def __init__(self, parent: RosboardClientGui):
        super(QWidget, self).__init__(parent)
        self.setObjectName("TopicsListWidget")
        self.setMinimumWidth(300)

        self.topic_btns = []

        self.ly_topics = QVBoxLayout()
        self.ly_topics.setAlignment(Qt.AlignTop)

        # Create the group of topic buttons
        self.topics_gb = QGroupBox()
        self.topics_gb.setObjectName("topics_gb")
        self.topics_gb.setLayout(self.ly_topics)
        self.topics_gb.setStyleSheet("QGroupBox#topics_gb{border: 1px solid black;}")

        scroll_area = QScrollArea()
        scroll_area.setWidget(self.topics_gb)
        scroll_area.setWidgetResizable(True)

        ly_main = QVBoxLayout(self)
        ly_main.addWidget(scroll_area)
        self.setLayout(ly_main)

    def add_topic(self, topic_name: str):
        """!
        Add a button to the topic list in the panel in alphabetic order.
        @param topic_name "str" name of the topic that will be linked to the button.
        """
        # Add topic to list and get index
        current_topics = [bt.text() for bt in self.topic_btns]
        current_topics.append(topic_name)
        current_topics = sorted(current_topics)
        topic_indx = current_topics.index(topic_name)

        # Insert the button into widget
        bt_topic = QPushButton(topic_name)
        bt_topic.clicked.connect(
            partial(self.parent().parent().parent().add_topic_to_panel, topic_name)
        )
        self.topic_btns.insert(topic_indx + 1, bt_topic)
        self.ly_topics.insertWidget(topic_indx, bt_topic)

    def add_topic_at_end(self, topic_name: str):
        """! Add a button to the at the end of the topic list.
        @param topic_name "str" name of the topic that will be linked to the button.
        """
        bt_topic = QPushButton(topic_name)
        bt_topic.clicked.connect(
            partial(self.parent().parent().parent().add_topic_to_panel, topic_name)
        )
        self.topic_btns.append(bt_topic)
        self.ly_topics.addWidget(self.topic_btns[-1])

    def remove_topic(self, topic_name: str):
        """! Remove a topic button from the list.
        @param topic_name "str" name of the topic whose button will be removed.
        """
        for button in self.topic_btns:
            if button.text() == topic_name:
                self.topic_btns.remove(button)
                button.deleteLater()

    def remove_all_topics(self):
        """! Remove all topics from widget."""
        topic_names = [btn.text() for btn in self.topic_btns]
        for tn in topic_names:
            self.remove_topic(tn)

    def get_current_topics(self) -> list:
        """!
        Return the current topics in widget.
        """
        return [btn.text() for btn in self.topic_btns]


class TopicsPanelWidget(QWidget):
    """!
    Widget that contains the required elements to connect to websocket.
    """

    def __init__(self, parent: RosboardClientGui):
        super(QWidget, self).__init__(parent)
        self.setObjectName("TopicsPanelWidget")
        self.setMinimumWidth(300)

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
        """!
        Add topic to panel and configure the layout.
        @param topic_name "str" name of the topic that will be added.
        """
        topic_wg = TopicWidget(self, topic_name)
        self.widgets_list.append(topic_wg)
        self.configure_panel()

    def remove_topic(self, topic_name: str):
        """!
        Removes a topic from the panel and configure the layout.
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
        """!
        Update the statistics for topics in panel.
        @param topic_stats "dict" dictionary with the topic stats. Dictionary
            key is topic name and value is a list with topic statistics.
        """
        for widget in self.widgets_list:
            stats = topic_stats[widget.topic_name]
            widget.update_topic_stats(stats[0], stats[1])

    def update_topic_state(self, topic_state: dict):
        for widget in self.widgets_list:
            state = topic_state[widget.topic_name]
            widget.update_topic_state(state)

    def configure_panel(self):
        count = 0
        for widget in self.widgets_list:
            self.ly_widget.addWidget(
                widget, count // self.MAX_COLS, count % self.MAX_COLS
            )
            count += 1

    def get_current_topics(self) -> list:
        """!
        Return the current topics in widget.
        """
        return [topic_wg.topic_name for topic_wg in self.widgets_list]


class TopicWidget(QWidget):
    """!
    Widget that will show the topic information. The widget includes the
    topic name, received frequency and time delay.
    @param topic_name "str"
    """

    TOPIC_STATE_DICT = {
        "DELAY": "QWidget#TopicWidget{background-color: #FF6666;}",
        "NORMAL": "QWidget#TopicWidget{background-color: #77CC66;}",
        "NO_DATA": "QWidget#TopicWidget{background-color: #B0B0B0;}",
    }

    def __init__(self, parent: TopicsPanelWidget, topic_name: str):
        super(QWidget, self).__init__(parent)
        self.setObjectName("TopicWidget")
        self.setAttribute(Qt.WA_StyledBackground)

        self.topic_name = topic_name

        bt_close = QPushButton("X")
        bt_close.clicked.connect(
            partial(
                self.parent().parent().parent().parent().add_topic_to_list,
                self.topic_name,
            )
        )

        lb_name = QLabel(self.topic_name)
        lb_name.setObjectName("TopicName")

        ly_top = QHBoxLayout()
        ly_top.setSpacing(5)
        ly_top.setContentsMargins(0, 0, 0, 0)
        ly_top.addWidget(lb_name, stretch=100)
        ly_top.addWidget(bt_close, stretch=1, alignment=Qt.AlignRight)

        self.freq_lb = QLabel("XXX.XX")
        self.latency_lb = QLabel("XXX.XX")

        ly_widget = QGridLayout()
        ly_widget.setSpacing(0)
        ly_widget.addLayout(ly_top, 0, 0, 1, 2)
        ly_widget.addWidget(QLabel("Freq. [Hz]:"), 1, 0)
        ly_widget.addWidget(QLabel("DT [ms]:"), 2, 0)
        ly_widget.addWidget(self.freq_lb, 1, 1, Qt.AlignRight)
        ly_widget.addWidget(self.latency_lb, 2, 1, Qt.AlignRight)
        self.setLayout(ly_widget)

    def update_topic_stats(self, frequency: float, latency: float):
        """!
        Update the topic statistics: frequency and latency.
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
        self.setStyleSheet(TopicWidget.TOPIC_STATE_DICT[state])


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    ui = RosboardClientGui()
    ui.showMaximized()
    sys.exit(app.exec())
