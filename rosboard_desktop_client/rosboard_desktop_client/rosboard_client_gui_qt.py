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
import sys

import rclpy
from rclpy.node import Node

from icmplib import ping
from time import time, sleep
from threading import Thread
from psutil import cpu_percent, net_io_counters
from socket import socket, AF_INET, SOCK_STREAM, gaierror

from functools import partial
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGridLayout, QLineEdit, QPushButton, QScrollArea, QGroupBox, QMessageBox

from rosboard_desktop_client.networking import RosboardClient
from rosboard_desktop_client.republishers import PublisherManager


class TopicHandler:

    ALPHA = 2.0 / (20.0 + 1.0)
    C_ALPHA = 1 - ALPHA

    def __init__(self, topic_name, client, node: Node):
        """! Class to handle the node behavior. 
        This class provides statistics measurements for topics such as the
        current latency and publish rate.

        @param topic_name "str" name of the topic being handled.
        @param client "RosboardClient" client for the topic.
        @param node "Node" ROS node instance to communicate with ROS.
        """

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
            parent_node=node, topic_name=topic_name, topic_class_name=message_type
        )
        self.client.create_socket_subscription(
            message_type, topic_name, self.topic_callback
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
        self.running = False

    def close_connection(self):
        """! Close the connection to the topic."""
        rclpy.logging.get_logger("rosboard_desktop_client").info(f"Closing connection for {self.topic_name}")
        self.client.destroy_socket_subscription(self.topic_name)
        self.running = False

    def define_node_state(self):
        while self.running:
            if self.t_last_msg is not None:
                if time() - self.t_last_msg > 5.0:
                    self.state = "NO_DATA"
                elif self.rate < 0.8 * self.avg_rate:
                    self.state = "DELAY"
                else:
                    self.state = "NORMAL"
            sleep(0.25)

    def calculate_average(self, time_list):
        """!
        Calculate the exponentially weighted moving average.
        @param time_list "list" contains the latest time values between messages.
        @return "float" with the average value. Return 0.0 if time_list has no values.
        """
        average = 0.0
        list_size = len(time_list)
        if list_size > 0:
            average = time_list[0]
            if list_size > 1:
                for indx in range(1, list_size):
                    average = TopicHandler.ALPHA * time_list[indx] + TopicHandler.C_ALPHA * average
        return average

    def topic_callback(self, msg):
        """!
        Topic callback function for incoming rosboard messages.

        This function processes the incoming messages in order to provide the
        stats of the 

        """
        self.n_msgs += 1
        t_current_msg = time()
        
        if self.t_last_msg is not None:
            if self.has_header:
                t_send = self.timestamp_to_secs(msg[1]["header"]["stamp"])
                self.latency_list.append(t_current_msg - t_send)
                self.latency_list = self.latency_list[-20:]
            
            self.rate_list.append(t_current_msg - self.t_last_msg)
            self.rate_list = self.rate_list[-20:]

        else:
            self.has_header = "header" in msg[1].keys()

        # Always execute this code block
        self.t_last_msg = t_current_msg
        self.republisher.parse_and_publish(msg)

    def timestamp_to_secs(self, header_stamp: float):
        """!
        Convert a header timestamp to a float value including nanoseconds.
        @param header_stamp "float" header with fields using for converting the
            timestamp to seconds.
        @return "float" value with the header timestamp.
        """
        return header_stamp["sec"] + header_stamp["nanosec"] * 10 ** -9

    def get_topic_stats(self):
        """!
        Calculate and return the topic stats.
        @return "list" includes three (3) values: the rate in which the latest
            messages were received; the latency if the message includes a header, and
            a boolean indicating if the message has header.
        """
        t_average = self.calculate_average(self.rate_list)
        self.rate = 1.0 / t_average if t_average != 0.0 else 0.0
        self.latency = self.calculate_average(self.latency_list) if self.has_header else 0.0
        return [self.rate, self.latency, self.has_header]


class RosboardClientGui(QMainWindow):

    def __init__(self):
        super(QMainWindow, self).__init__()

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
        ly_bottom = QHBoxLayout()
        ly_bottom.addWidget(self.topics_list_widget)
        ly_bottom.addWidget(self.topics_panel_widget)

        # Define the main layout for window
        ly_main = QVBoxLayout()
        ly_main.addLayout(ly_top)
        ly_main.addLayout(ly_bottom)
        
        # Define the central widget and set the layout
        wg_main = QWidget(self)
        wg_main.setLayout(ly_main)
        self.setCentralWidget(wg_main)

        # Declare the variables to store stats
        self.cpu_usage = 0.0
        self.roundtrip = 0.0
        self.download_speed = 0.0
        self.old_bytes_recv = 0.0

        # Create timers to update each of the stats
        cpu_usage_timer = QTimer(self)
        roundtrip_timer = QTimer(self)
        download_speed_timer = QTimer(self)
        cpu_usage_timer.timeout.connect(self.update_cpu_usage)
        roundtrip_timer.timeout.connect(self.update_roundtrip)
        download_speed_timer.timeout.connect(self.update_download_speed)
        cpu_usage_timer.start(250)
        roundtrip_timer.start(250)
        download_speed_timer.start(250)

        # Create a timer to update the stats panel
        stats_timer = QTimer(self)
        stats_timer.timeout.connect(self.update_stats)
        stats_timer.start(250)

        # Create a timer to update the topic stats
        self.topic_stats_timer = QTimer(self)
        self.topic_stats_timer.timeout.connect(self.update_topic_stats_and_state)

        # Timer that checks the connection status to handle closures
        networking_timer = QTimer(self)
        networking_timer.timeout.connect(self.check_connection_status)
        networking_timer.start(500)

        # Timer to update the available topics
        self.topic_upd_timer = QTimer(self)
        self.topic_upd_timer.timeout.connect(self.update_available_topics)

    def closeEvent(self, event):
        """! Function for handling the close interface. """
        RosboardClient.stop_reactor()
        if self.client is not None:
            if self.client.protocol.is_connected:
                self.disconnect_from_server()
        super(QMainWindow, self).closeEvent(event)

    def reset_network_attributes(self):
        """! Function that resets the network-related attributes of the class."""
        self.client = None
        self.server_ip_addr = None
        self.is_connected = False
        self.retry_connection = False
        self.topic_handlers = {}
        self.available_topics = []

    def show_warning_message(self, title, message):
        """! Function to generate a warning in the interface.
        @param title "str" title for the warning.
        @param message "str" content for the warning.
        """
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setText(message)
        msg.setWindowTitle(title)
        msg.exec_()

    def check_connection_status(self):
        """! Check the connection status of the client.
        """
        if self.client is not None:
            self.is_connected = self.client.protocol.is_connected
            if self.retry_connection and self.is_connected:
                self.restore_interface()
                self.retry_connection = False
            else:
                self.retry_connection = not self.is_connected

    def restore_interface(self):
        """!
        Restore the interface to the current state after a reconnection.
        """
        self.node.get_logger().info("Restoring interface after reconnection.")
        # Get current topics to restore them later.
        current_topics = []
        for tw in self.topics_panel_widget.widgets_list:
            current_topics.append(tw.topic_name)
        # Delete the topic handlers # TODO: convert this into a function. SRP.
        for topic in list(self.topic_handlers.keys()):
            self.topic_handlers[topic].close_connection()
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
        self.cpu_usage = cpu_percent()

    def update_roundtrip(self):
        if self.client is not None:
            if self.client.protocol.is_connected:
                ping_response = ping(address=self.server_ip_addr, count=1, timeout=0.5, privileged=False)
                self.roundtrip = ping_response.avg_rtt
        else:
            self.roundtrip = 0.0

    def update_download_speed(self):
        net_if_stats_val = net_io_counters()
        received_bytes = net_if_stats_val.bytes_recv - self.old_bytes_recv
        self.old_bytes_recv = net_if_stats_val.bytes_recv
        self.download_speed = (received_bytes / 1024.0) / 0.25

    def update_stats(self):
        self.stats_widget.update_stats_widget(
            self.cpu_usage,
            self.roundtrip,
            self.download_speed
        )

    def update_available_topics(self):
        """! Updates the available topics in the interface.

        This function checks for the currently available topics from the
        server. 

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

    def test_connection(self):
        """!
        Check the websocket status and enable the connect button.
        """
        try:
            ip_addr, port = self.connection_widget.get_connection_address()
            test_socket = socket(AF_INET, SOCK_STREAM)
            test_socket.settimeout(1.0)
            is_avail = test_socket.connect_ex((ip_addr, int(port))) == 0
            test_socket.close()
            if not is_avail:
                self.show_warning_message(
                    "Can not connect to server!",
                    "Connection error!"
                )
            return is_avail
        except ValueError as e:
            self.show_warning_message(
                "Target web socket is not enabled. Is the port correct?",
                "Target web socket is not enabled!"
            )
            return False
        except gaierror as e:
            self.show_warning_message(
                "Target web socket is not enabled!",
                "Target web socket is not enabled. Is the address correct?"
            )
            return False

    def connect_to_server(self):
        """! Function to connect to rosboard server.

        This function will attempt to connect to the rosboard server. In order
        to do so, it will first test the connection based on the parameters
        that were input in the connection address field. Then, it will create
        a RosboardClient instance and configure the interface according to the
        current available topics.
        """
        # Test the connection before connecting.
        if self.test_connection():

            # Get the connection parameters from the connection widget.
            ip_addr, port = self.connection_widget.get_connection_address()

            # Connect to the rosboard client
            self.client = RosboardClient(
                host=f"{ip_addr}:{port}", 
                connection_timeout=5.0
            )

            # Get topics list and add them to the topics list widget.
            self.topic_handlers = {}
            self.available_topics = self.client.get_available_topics()
            for topic in self.available_topics:
                self.topics_list_widget.add_topic_at_end(topic)

            # Start the timers to update topics list and stats.
            self.topic_stats_timer.start(250)
            self.topic_upd_timer.start(5000)

            # Store the connection address and set flag
            self.server_ip_addr = ip_addr
            self.is_connected = True

            # Configure the connection widget to connected status
            self.connection_widget.set_buttons_status(self.is_connected)
            self.connection_widget.toggle_edits(False)

    def disconnect_from_server(self):
        """! Disconnects the interface from the rosboard server.

        This function will destroy the socket connections to topics in the
        server, destroy the connection and restore the interface to its 
        disconnected status.
        """
        # Stop timers to update topics list and stats.
        self.topic_upd_timer.stop()
        self.topic_stats_timer.stop()

        # Destroy the socket connections for each topic
        for topic in list(self.topic_handlers.keys()):
            self.topic_handlers[topic].close_connection()
            del self.topic_handlers[topic]

        # Reset the interface network attributes
        self.reset_network_attributes()

        # Restore the interface to the disconnected status
        self.topics_list_widget.remove_all_topics()
        self.topics_panel_widget.remove_all_topics()
        self.connection_widget.toggle_edits(True)
        self.connection_widget.set_buttons_status(self.is_connected)

    def add_topic_to_panel(self, topic_name):
        try:
            self.topic_handlers[topic_name] = TopicHandler(topic_name, self.client, self.node)
            self.topics_list_widget.remove_topic(topic_name)
            self.topics_panel_widget.add_topic(topic_name)
        except ModuleNotFoundError as e:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Can not load topic message type!")
            msg.setWindowTitle("Can not load message!")
            msg.exec_()

    def add_topic_to_list(self, topic_name):
        if topic_name in self.topic_handlers.keys():
            self.topic_handlers[topic_name].close_connection()
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
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Create the line edits for the IP Address and the port
        self.address_le = QLineEdit()

        # Define a default address
        self.address_le.setText("localhost:8888")

        # Create the buttons for connecting and disconnecting the buttons
        self.connect_bt = QPushButton("CONNECT")
        self.connect_bt.setEnabled(True)
        self.disconnect_bt = QPushButton("DISCONNECT")
        self.disconnect_bt.setEnabled(False)

        # Connect to slots in parent
        self.connect_bt.clicked.connect(self.parent().connect_to_server)
        self.disconnect_bt.clicked.connect(self.parent().disconnect_from_server)

        # Define the widget layout
        ly_widget = QGridLayout()
        ly_widget.setSpacing(0)
        ly_widget.setContentsMargins(0, 0, 0, 0)        
        ly_widget.addWidget(QLabel("ADDRESS:"), 0, 0, Qt.AlignRight)
        ly_widget.addWidget(self.address_le, 0, 1, 1, 2)
        ly_widget.addWidget(self.connect_bt, 2, 1)
        ly_widget.addWidget(self.disconnect_bt, 2, 2)
        self.setLayout(ly_widget)

    def get_connection_address(self):
        address = self.address_le.text().split("//")[-1]
        address = address.split(':')
        if len(address) == 2:
            return address[0], address[1]
        else:
            return "", ""

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


class StatsWidget(QWidget):
    """!
    Widget that contains elements to show the stats for the user.
    The stats consist of the CPU usage represented as a percentage,
    the round trip time to the connected socket, and the current
    download speed.
    """
    def __init__(self, parent):
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

    def update_stats_widget(self, cpu_usage, roundtrip, download):
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
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.setObjectName("TopicsListWidget")

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

    def add_topic(self, topic_name):
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
        bt_topic.clicked.connect(partial(self.parent().parent().add_topic_to_panel, topic_name))
        self.topic_btns.insert(topic_indx + 1, bt_topic)
        self.ly_topics.insertWidget(topic_indx, bt_topic)

    def add_topic_at_end(self, topic_name):
        """!
        add a button ot the topic list in the panel at its end.
        @param topic_name "str" name of the topic that will be linked to the button.
        """
        bt_topic = QPushButton(topic_name)
        bt_topic.clicked.connect(partial(self.parent().parent().add_topic_to_panel, topic_name))
        self.topic_btns.append(bt_topic)
        self.ly_topics.addWidget(self.topic_btns[-1])

    def remove_topic(self, topic_name):
        """!
        Remove a topic button from the list.
        """
        for button in self.topic_btns:
            if button.text() == topic_name:
                self.topic_btns.remove(button)
                button.deleteLater()

    def remove_all_topics(self):
        """!
        Remove all topics from widget.
        """
        topic_names = [btn.text() for btn in self.topic_btns]
        for tn in topic_names:
            self.remove_topic(tn)

    def get_current_topics(self):
        """!
        Return the current topics in widget.
        """
        return [btn.text() for btn in self.topic_btns]


class TopicsPanelWidget(QWidget):
    """!
    Widget that contains the required elements to connect to websocket.
    """
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        
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

    def add_topic(self, topic_name):
        topic_wg = TopicWidget(self, topic_name)
        self.widgets_list.append(topic_wg)
        self.configure_panel()

    def remove_topic(self, topic_name):
        for topic_wg in self.widgets_list:
            if topic_wg.topic_name == topic_name:
                self.widgets_list.remove(topic_wg)
                topic_wg.deleteLater()
        self.configure_panel()

    def remove_all_topics(self):
        topic_names = [topic_wg.topic_name for topic_wg in self.widgets_list]
        for tn in topic_names:
            self.remove_topic(tn)

    def update_topic_stats(self, topic_stats):
        for widget in self.widgets_list:
            stats = topic_stats[widget.topic_name]
            widget.update_topic_stats(stats[0], stats[1], stats[2])

    def update_topic_state(self, topic_state):
        for widget in self.widgets_list:
            state = topic_state[widget.topic_name]
            widget.update_topic_state(state)

    def configure_panel(self):
        count = 0
        for widget in self.widgets_list:
            self.ly_widget.addWidget(widget, count // 4, count % 4)
            count += 1

    def get_current_topics(self):
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
    def __init__(self, parent, topic_name):
        super(QWidget, self).__init__(parent)
        self.setObjectName("TopicWidget")
        # TODO: remove stylesheet and add to .qss file.
        #self.setAutoFillBackground(True)
        self.setAttribute(Qt.WA_StyledBackground)
        self.setStyleSheet("QWidget#TopicWidget {border: 2px solid black;}")

        self.topic_name = topic_name

        self.freq_lb = QLabel("XXX.XX")
        self.latency_lb = QLabel("XXX.XX")
        
        bt_close = QPushButton("X")
        bt_close.clicked.connect(partial(
            self.parent().parent().parent().add_topic_to_list,
            self.topic_name
        ))

        lb_name = QLabel(self.topic_name)

        ly_widget = QGridLayout()
        ly_widget.setSpacing(0)
        ly_widget.addWidget(lb_name, 1, 0, 1, 2)
        ly_widget.addWidget(QLabel("Freq. [Hz]:"), 2, 0)
        ly_widget.addWidget(QLabel("DT [ms]:"), 3, 0)
        ly_widget.addWidget(self.freq_lb, 2, 1, Qt.AlignRight)
        ly_widget.addWidget(self.latency_lb, 3, 1, Qt.AlignRight)
        ly_widget.addWidget(bt_close, 0, 0, 1, 2)
        self.setLayout(ly_widget)

    def update_topic_stats(self, frequency, latency, has_latency=True):
        """!
        Update the topic statistics: frequency and latency.
        @param frequency "float" value.
        @param latency "float" value.
        @param has_latency "bool" flag to indicate wheter the topic has 
        latency calculations or not.
        """
        self.freq_lb.setText(f"{frequency:3.2f}")
        if has_latency:
            self.latency_lb.setText(f"{latency * 1000:3.2f}")
        else:
            self.latency_lb.setText("N/A")

    def update_topic_state(self, state):
        """!
        Update the node element color depending on topic state.
        @param state "str" represent the received messages values for topic.
        """
        if state == "DELAY":
            self.setStyleSheet("QWidget#TopicWidget{border: 2px solid black; background: red;}")
        elif state == "NORMAL":
            self.setStyleSheet("QWidget#TopicWidget{border: 2px solid black; background: green;}")
        else:
            self.setStyleSheet("QWidget#TopicWidget{border: 2px solid black; background: gray;}")


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    ui = RosboardClientGui()
    ui.showMaximized()
    sys.exit(app.exec())