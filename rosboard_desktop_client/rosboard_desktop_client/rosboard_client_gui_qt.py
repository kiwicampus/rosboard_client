
import sys

import rclpy
from rclpy.node import Node

from icmplib import ping
from psutil import cpu_percent, net_io_counters
from socket import socket, AF_INET, SOCK_STREAM

from functools import partial
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGridLayout, QLineEdit, QPushButton, QScrollArea, QGroupBox

from rosboard_desktop_client.networking import RosboardClient

class RosboardClientGui(QMainWindow):

    def __init__(self):
        super(QMainWindow, self).__init__()

        # Create the node 
        self.node = Node("rosboard_desktop_gui")

        # Relevant variables
        self.client = None
        self.server_ip_addr = None
        self.is_connected = False

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
        cpu_usage_timer.start(100)
        roundtrip_timer.start(100)
        download_speed_timer.start(100)

        # Create a timer to update the stats panel
        stats_timer = QTimer(self)
        stats_timer.timeout.connect(self.update_stats)
        stats_timer.start(250)

        # Create the timer to update the connection buttons
        conn_timer = QTimer(self)
        conn_timer.timeout.connect(self.check_websocket_status)
        conn_timer.start(100)

    def update_cpu_usage(self):
        self.cpu_usage = cpu_percent()

    def update_roundtrip(self):
        if self.is_connected:
            ping_response = ping(address=self.server_ip_addr, count=1, timeout=0.5, privileged=False)
            self.roundtrip = ping_response.avg_rtt
        else:
            self.roundtrip = 0.0

    def update_download_speed(self):
        net_if_stats_val = net_io_counters()
        received_bytes = net_if_stats_val.bytes_recv - self.old_bytes_recv
        self.old_bytes_recv = net_if_stats_val.bytes_recv
        self.download_speed = (received_bytes / 1024.0) / 0.5

    def update_stats(self):
        self.stats_widget.update_stats_widget(
            self.cpu_usage,
            self.roundtrip,
            self.download_speed
        )

    def check_websocket_status(self):
        ip_addr, port = self.connection_widget.get_connection_address()
        try:
            test_socket = socket(AF_INET, SOCK_STREAM)
            test_socket.settimeout(0.5)
            is_avail = test_socket.connect_ex((ip_addr, int(port))) == 0
            self.connection_widget.set_buttons_status(is_avail, self.is_connected)
            test_socket.close()
        except ValueError as e:
            pass

    def connect_to_server(self):
        ip_addr, port = self.connection_widget.get_connection_address()

        # Connect to client
        self.client = RosboardClient(
            host=f"{ip_addr}:{port}", 
            connection_timeout=5.0
        )

        # Get topics list and add them to interface
        topics_list = self.client.get_available_topics()
        for topic in topics_list:
            self.topics_list_widget.add_topic(topic)

        self.connection_widget.toggle_edits(False)
        self.server_ip_addr = ip_addr
        self.is_connected = True

    def disconnect_from_server(self):
        self.client = None
        self.connection_widget.toggle_edits(True)
        self.is_connected = False

    def add_topic_to_panel(self, topic_name):
        self.topics_list_widget.remove_topic(topic_name)
        self.topics_panel_widget.add_topic(topic_name)

    def add_topic_to_list(self, topic_name):
        self.topics_panel_widget.remove_topic(topic_name)
        self.topics_list_widget.add_topic(topic_name)


class ConnectionWidget(QWidget):
    """!
    Widget that contains the required elements to connect to websocket.
    """
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Create the line edits for the IP Address and the port
        self.ip_address_le = QLineEdit()
        self.port_le = QLineEdit()

        # Define a default address
        self.ip_address_le.setText("localhost")
        self.port_le.setText("8888")

        # Create the buttons for connecting and disconnecting the buttons
        self.connect_bt = QPushButton("CONNECT")
        self.connect_bt.setEnabled(False)
        self.disconnect_bt = QPushButton("DISCONNECT")
        self.disconnect_bt.setEnabled(False)

        # Connect to slots in parent
        self.connect_bt.clicked.connect(self.parent().connect_to_server)
        self.disconnect_bt.clicked.connect(self.parent().disconnect_from_server)

        ip_addr_lb = QLabel("IP ADDRESS")
        ip_addr_lb.adjustSize()

        # Define the widget layout
        ly_widget = QGridLayout()
        ly_widget.setSpacing(0)
        ly_widget.setContentsMargins(0, 0, 0, 0)
        ly_widget.addWidget(ip_addr_lb, 0, 0, Qt.AlignHCenter)
        ly_widget.addWidget(QLabel("PORT"), 0, 1, Qt.AlignHCenter)
        ly_widget.addWidget(self.ip_address_le, 1, 0)
        ly_widget.addWidget(self.port_le, 1, 1)
        ly_widget.addWidget(self.connect_bt, 2, 0)
        ly_widget.addWidget(self.disconnect_bt, 2, 1)
        self.setLayout(ly_widget)

    def get_connection_address(self):
        ip_address = self.ip_address_le.text()
        port = self.port_le.text()
        return ip_address, port

    def toggle_edits(self, enabled):
        self.ip_address_le.setEnabled(enabled)
        self.port_le.setEnabled(enabled)

    def set_buttons_status(self, is_avail, is_connected):
        self.connect_bt.setEnabled(is_avail and not is_connected)
        self.disconnect_bt.setEnabled(is_connected)


class StatsWidget(QWidget):
    """!
    Widget that contains elements to show the stats for the user.
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

        self.topic_btns = []
        self.ly_topics = QVBoxLayout()

        # Create the group of topic buttons
        self.topics_gb = QGroupBox()
        self.topics_gb.setLayout(self.ly_topics)

        scroll_area = QScrollArea()
        scroll_area.setWidget(self.topics_gb)
        scroll_area.setWidgetResizable(True)

        ly_main = QVBoxLayout(self)
        ly_main.addWidget(scroll_area)

        self.setLayout(ly_main)

    def add_topic(self, topic_name):
        """!
        Add a button to the topic list in the panel.
        @param topic_name "str" name of the topic that will be linked to the button. 
        """
        bt_topic = QPushButton(topic_name)
        bt_topic.clicked.connect(partial(self.parent().parent().add_topic_to_panel, topic_name))
        self.topic_btns.append(bt_topic)
        self.ly_topics.addWidget(self.topic_btns[-1])

    def remove_topic(self, topic_name):
        """!
        Remove the last added button of the topic list.
        """
        for button in self.topic_btns:
            if button.text() == topic_name:
                self.topic_btns.remove(button)
                button.deleteLater()


class TopicsPanelWidget(QWidget):
    """!
    Widget that contains the required elements to connect to websocket.
    """
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        
        # Define the layout for the widget
        self.ly_widget = QGridLayout()

        # 
        topics_gb = QGroupBox()
        topics_gb.setLayout(self.ly_widget)
        
        # Create the scroll area
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

    def configure_panel(self):
        count = 0
        for widget in self.widgets_list:
            self.ly_widget.addWidget(widget, count // 4, count % 4)
            count += 1


class TopicWidget(QWidget):
    """!
    Widget that will show the topic information.
    """
    def __init__(self, parent, topic_name):
        super(QWidget, self).__init__(parent)

        self.topic_name = topic_name

        self.freq_lb = QLabel("XXX.XX")
        self.latency_lb = QLabel("XXX.XX")
        
        bt_close = QPushButton("X")
        bt_close.clicked.connect(partial(
            self.parent().parent().parent().add_topic_to_list,
            self.topic_name
        ))

        ly_widget = QGridLayout()
        ly_widget.addWidget(QLabel(self.topic_name), 0, 0, 1, 2)
        ly_widget.addWidget(QLabel("Freq. [Hz]:"), 1, 0)
        ly_widget.addWidget(QLabel("DT [ms]:"), 2, 0)
        ly_widget.addWidget(self.freq_lb, 1, 1, Qt.AlignRight)
        ly_widget.addWidget(self.latency_lb, 2, 1, Qt.AlignRight)
        ly_widget.addWidget(bt_close, 0, 2, 3, 1)

        self.setLayout(ly_widget)


def main():
    # 
    rclpy.init(args=sys.argv)


    app = QApplication(sys.argv)
    ui = RosboardClientGui()
    ui.showMaximized()
    sys.exit(app.exec())