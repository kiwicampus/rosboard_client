
import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGridLayout, QLineEdit, QPushButton, QScrollArea, QGroupBox


class RosboardClientGui(QMainWindow):

    def __init__(self):
        super(QMainWindow, self).__init__()

        # Main window configurations
        self.setWindowTitle("Rosboard Client GUI")

        # Initialize custom widgets
        self.connection_widget = ConnectionWidget(self)
        self.stats_widget = StatsWidget(self)
        self.topics_list_widget = TopicsListWidget(self)
        self.topics_panel_widget = TopicsPanelWidget(self)

        # Define the top layout (connection + stats)
        ly_top = QHBoxLayout()
        ly_top.addWidget(self.connection_widget, 8)
        ly_top.addWidget(self.stats_widget, 2)

        # Define the bottom layout (topics list + topic stats)
        ly_bottom = QHBoxLayout()
        ly_bottom.addWidget(self.topics_list_widget, 2)
        ly_bottom.addWidget(self.topics_panel_widget, 8)

        # Define the main layout for window
        ly_main = QVBoxLayout()
        ly_main.addLayout(ly_top, 1)
        ly_main.addLayout(ly_bottom, 3)
        
        # Define the central widget and set the layout
        wg_main = QWidget(self)
        wg_main.setLayout(ly_main)
        self.setCentralWidget(wg_main)


class ConnectionWidget(QWidget):
    """!
    Widget that contains the required elements to connect to websocket.
    """
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        # Create the line edits for the IP Address and the port
        self.ip_address_le = QLineEdit()
        self.port_le = QLineEdit()

        # Create the buttons for connecting and disconnecting the buttons
        self.connect_bt = QPushButton("CONNECT")
        self.disconnect_bt = QPushButton("DISCONNECT")

        # Define the widget layout
        ly_widget = QGridLayout()
        ly_widget.addWidget(QLabel("IP ADDRESS"), 0, 0, Qt.AlignCenter)
        ly_widget.addWidget(QLabel("PORT"), 0, 1, Qt.AlignCenter)
        ly_widget.addWidget(self.ip_address_le, 1, 0)
        ly_widget.addWidget(self.port_le, 1, 1)
        ly_widget.addWidget(self.connect_bt, 2, 0)
        ly_widget.addWidget(self.disconnect_bt, 2, 1)
        self.setLayout(ly_widget)

class StatsWidget(QWidget):
    """!
    Widget that contains elements to show the stats for the user.
    """
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        
        # Define the labels to store the stats. values.
        self.cpu_usage_lb = QLabel("XXX.XX")
        self.roundtrip_lb = QLabel("XXX.XX")
        self.download_lb = QLabel("XXX.XX")

        # Define the widget layout
        ly_widget = QGridLayout()
        ly_widget.addWidget(QLabel("CPU USAGE:"), 0, 0)
        ly_widget.addWidget(QLabel("ROUNDTRIP:"), 1, 0)
        ly_widget.addWidget(QLabel("DOWNLOAD SPEED:"), 2, 0)
        ly_widget.addWidget(self.cpu_usage_lb, 0, 1)
        ly_widget.addWidget(self.roundtrip_lb, 1, 1)
        ly_widget.addWidget(self.download_lb, 2, 1)
        ly_widget.addWidget(QLabel("%"), 0, 2)
        ly_widget.addWidget(QLabel("ms"), 1, 2)
        ly_widget.addWidget(QLabel("kbps"), 2, 2)
        self.setLayout(ly_widget)
        

class TopicsListWidget(QWidget):
    """!
    Widget that contains the required elements to connect to websocket.
    """
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        add_topic_bt = QPushButton("Add Topic")
        add_topic_bt.clicked.connect(self.add_topic)

        remove_topic_bt = QPushButton("Remove Topic")
        remove_topic_bt.clicked.connect(self.remove_topic)

        self.index = 0
        self.topic_btns = []

        self.ly_topics = QVBoxLayout()

        # Create the group of topic buttons
        self.topics_gb = QGroupBox()
        self.topics_gb.setLayout(self.ly_topics)

        scroll_area = QScrollArea()
        scroll_area.setWidget(self.topics_gb)
        scroll_area.setWidgetResizable(True)

        ly_main = QVBoxLayout(self)
        ly_main.addWidget(add_topic_bt)
        ly_main.addWidget(remove_topic_bt)
        ly_main.addWidget(scroll_area)

        self.setLayout(ly_main)

    def add_topic(self):
        """!
        Add a button to the topic list in the panel.
        @param topic_name "str" name of the topic that will be linked to the button. 
        """
        self.topic_btns.append(QPushButton(f"Topic {self.index}"))
        self.index += 1
        self.ly_topics.addWidget(self.topic_btns[-1])

    def remove_topic(self):
        """!
        Remove the last added button of the topic list.
        """
        if self.index > 0:
            self.index -= 1
            bt_to_be_removed = self.topic_btns.pop(self.index)
            bt_to_be_removed.deleteLater()


class TopicsPanelWidget(QWidget):
    """!
    Widget that contains the required elements to connect to websocket.
    """
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        
        ly_widget = QGridLayout()

        for i in range(4):
            for j in range(4):
                topic_wg = TopicWidget(self, f"Topic {i}, {j}")
                ly_widget.addWidget(topic_wg, i, j)

        self.setLayout(ly_widget)

class TopicWidget(QWidget):
    """!
    Widget that will show the topic information.
    """
    def __init__(self, parent, name):
        super(QWidget, self).__init__(parent)

        self.freq_lb = QLabel("XXX.XX")
        self.latency_lb = QLabel("XXX.XX")
        bt_close = QPushButton("X")

        ly_widget = QGridLayout()
        ly_widget.addWidget(QLabel(name), 0, 0, 1, 2)
        ly_widget.addWidget(QLabel("Freq. [Hz]:"), 1, 0)
        ly_widget.addWidget(QLabel("DT [ms]:"), 2, 0)
        ly_widget.addWidget(self.freq_lb, 1, 1, Qt.AlignRight)
        ly_widget.addWidget(self.latency_lb, 2, 1, Qt.AlignRight)
        ly_widget.addWidget(bt_close, 0, 2, 3, 1)

        self.setLayout(ly_widget)


def main():
    app = QApplication(sys.argv)
    ui = RosboardClientGui()
    ui.showMaximized()
    sys.exit(app.exec())