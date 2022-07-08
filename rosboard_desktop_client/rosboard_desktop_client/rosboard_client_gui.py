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
import os
from turtle import width

from psutil import cpu_percent
from psutil import net_io_counters


from time import sleep, time
import tkinter as tk
from functools import partial
from pathlib import Path
from threading import Thread

from icmplib import ping
from socket import socket, AF_INET, SOCK_STREAM

import rclpy
from PIL import Image, ImageTk
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rosboard_desktop_client.networking import RosboardClient
from rosboard_desktop_client.republishers import PublisherManager
from rosboard_desktop_client.streamers import GenericStreamer


class ScrolledFrame(tk.Frame):
    def __init__(self, parent, *args, **kw):
        tk.Frame.__init__(self, parent, *args, **kw)

        vscrollbar = tk.Scrollbar(self, orient=tk.VERTICAL)
        vscrollbar.pack(fill=tk.Y, side=tk.RIGHT, expand=True)

        self.canvas = tk.Canvas(
            self, bd=0, highlightthickness=0, yscrollcommand=vscrollbar.set
        )
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vscrollbar.config(command=self.canvas.yview)
        self.canvas.xview_moveto(0)
        self.canvas.yview_moveto(0)

        self.container = tk.Frame(self.canvas)
        self.container_id = self.canvas.create_window(
            0, 0, window=self.container, anchor=tk.NW
        )
        self.container.bind("<Configure>", self._configure_container)
        self.canvas.bind("<Configure>", self._configure_canvas)

    def _configure_container(self, event):
        """!Configures the container.
            @param event: event that will be handled with this callback.
        """
        size = (self.container.winfo_reqwidth(), self.container.winfo_reqheight())
        self.canvas.config(scrollregion="0 0 {} {}".format(*size))
        if self.container.winfo_reqwidth() != self.canvas.winfo_width():
            self.canvas.config(width=self.container.winfo_reqwidth())
        if self.container.winfo_reqheight() != self.canvas.winfo_height():
            self.canvas.config(height=self.container.winfo_reqheight())

    def _configure_canvas(self, event):
        """!Configures the canvas height.
            @param event: event that will be handled with this callback.
        """
        if self.container.winfo_reqwidth() != self.canvas.winfo_width():
            self.canvas.itemconfigure(
                self.container_id, width=self.canvas.winfo_width()
            )


class Application:
    def __init__(self, root, node):

        self.node = node
        canvasmain = tk.Canvas(root)
        canvasmain.pack(fill="both", expand=True)

        this_path = Path(os.path.realpath(__file__))
        self.background_path = os.path.join(
            this_path.parent, "resources", "background.jpg"
        )
        img = ImageTk.PhotoImage(Image.open(self.background_path))
        canvasmain.create_image(0, 0, anchor="nw", image=img)

        def resize_image(e):
            global image, resized, image2
            # open image to resize it
            image = Image.open(self.background_path)
            # resize the image with width and height of root
            resized = image.resize((e.width, e.height), Image.ANTIALIAS)
            image2 = ImageTk.PhotoImage(resized)
            canvasmain.create_image(0, 0, image=image2, anchor="nw")

        canvasmain.bind("<Configure>", resize_image)
        canvasmain.xview_moveto(0)
        canvasmain.yview_moveto(0)

        self.master = canvasmain
        self.buttons = []
        self.data = []

        self.frame_conf = tk.Frame(self.master, background="light sky blue")
        self.frame_conf.pack()

        label1 = tk.Label(self.frame_conf, text="URL", background="light sky blue")
        label1.grid(row=1, column=0, columnspan=2, sticky="ew")

        self.entry = tk.Entry(self.frame_conf)
        self.entry.insert(tk.END, "localhost:8888")
        self.entry.grid(row=1, column=2, columnspan=3, sticky=tk.W + tk.E, padx=5)

        self.bt_conn = tk.Button(
            self.frame_conf,
            text="CONNECT",
            command=self.url,
            background="light sky blue",
        )
        self.bt_conn.grid(row=1, column=5, columnspan=2, sticky="ew")

        self.bt_alive = tk.Button(
            self.frame_conf, text="STATUS", bg="#90EE90", state=tk.DISABLED
        )
        # Light green #90EE90
        # Grey #D3D3D3
        # Light red #FF7F7F
        self.bt_alive.grid(row=1, column=9, columnspan=1, sticky="ew")

        # ---------------------------------------------------------------
        # CPU Label
        # ---------------------------------------------------------------
        # Create the string variable to update text
        self.cpu_label_txt = tk.StringVar()
        self.cpu_label_txt.set("CPU [%]: {:4.2f}".format(0.0))

        # Define the CPU usage label
        cpu_usage_lb = tk.Label(self.frame_conf, textvariable=self.cpu_label_txt, background="light sky blue")
        cpu_usage_lb.grid(row=0, column=10, columnspan=2, sticky="nsew", padx=5, pady=2)
        # ---------------------------------------------------------------

        # ---------------------------------------------------------------
        # Roundtrip label
        # ---------------------------------------------------------------
        # Variable to store if a connection is being stabblished
        self.is_connected = False

        # Variable to store the connection IP address
        self.server_ip_addr = ""

        # Create the string variable to update text
        self.rt_label_txt = tk.StringVar()
        self.rt_label_txt.set("Roundtrip [ms]: {:5.2f}".format(0.0))

        # Define the roundtrip label (from ping-ing the server IP address)
        roundtrip_lb = tk.Label(
            self.frame_conf, textvariable=self.rt_label_txt, background="light sky blue"
        )
        roundtrip_lb.grid(row=1, column=10, columnspan=2, sticky="nsew", padx=5, pady=2)
        # ---------------------------------------------------------------

        # ---------------------------------------------------------------
        # Download speed
        # ---------------------------------------------------------------
        # Create the string variable to update text
        self.net_ds_txt = tk.StringVar()
        self.net_ds_txt.set("Download [MB/s]: {:4.2f}".format(0.0))

        # Define the download speed label (from 'docker0' network interface)

        download_lb = tk.Label(
            self.frame_conf, textvariable=self.net_ds_txt, background="light sky blue"
        )
        download_lb.grid(row=2, column=10, columnspan=2, sticky="nsew", padx=5, pady=2)

        self.frame_stats = tk.Frame(self.master, background="light sky blue")
        self.frame_stats.pack()

        self.frame_buttons = ScrolledFrame(self.frame_stats)
        self.frame_buttons.pack(side="left")

        self.datainfogen = tk.Frame(self.frame_stats, background="light sky blue")
        self.datainfogen.pack(side="right", padx=116, ipady=10, anchor="nw")

        self.NUM_ROWS = 5
        self.NUM_COLS = 3
        self.SELECT_TOPICS = 0


        # ---------------------------------------------------------------
        # CPU Label
        # ---------------------------------------------------------------
        # Define the GUI running variable to execute thread as a decent person
        self.is_gui_running = True

        # Create a thread to update the user interface as a decent person
        self.update_gui_th = Thread(target=self.update_gui_cb, daemon=True)
        self.update_gui_th.start()
        # ---------------------------------------------------------------

    def update_gui_cb(self):

        # Define a variable to store the last received packages count
        old_bytes_recv = 0

        # Create an iteration variable to calculate download speed
        download_speed_count = 0

        conn_test_count = 0

        # Runs while interface is running
        while self.is_gui_running:

            # Executes if interface is not connected to socket
            if not self.is_connected and conn_test_count == 4:
                
                # Test the connection to the IP field
                is_alive = self.test_socket_status()

                # Configures the interface depending on server result
                if is_alive:
                    self.bt_conn.config(state=tk.NORMAL)
                else:
                    self.bt_conn.config(state=tk.DISABLED)

                conn_test_count = 0

            elif conn_test_count < 4:
                conn_test_count += 1

            # Executes if connected to socket
            else:

                # Disables connection button
                self.bt_conn.config(state=tk.DISABLED)

                # Get the roundtrip value from the server
                ping_response_val = ping(address=self.server_ip_addr, count=1, timeout=0.5, privileged=False)

                # Update the roundtrip value to the server
                self.rt_label_txt.set("Roundtrip [ms]: {:5.2f}".format(ping_response_val.avg_rtt))

            # Get the process CPU utilization
            cpu_percent_val = cpu_percent()

            # Update the CPU utilization text variable
            self.cpu_label_txt.set("CPU [%]: {:4.2f}".format(cpu_percent_val))

            # Check if iterator meets condition
            if download_speed_count == 4:

                # Get the current networks interfaces statistics
                net_if_stats_val = net_io_counters()

                # Calculate the received packages within loop
                received_bytes = net_if_stats_val.bytes_recv - old_bytes_recv

                # Update the old packages count
                old_bytes_recv = net_if_stats_val.bytes_recv

                # Convert to MB/s
                net_if_download_speed = (received_bytes / 1024.0 / 1024.0) / 0.5

                # Update the download speed text variable
                self.net_ds_txt.set("Download [MiB/s]: {:4.2f}".format(net_if_download_speed))

                # Reset counter
                download_speed_count = 0

            # Update the iterator counter
            download_speed_count += 1

            # Sleep to avoid being inefficient (a.k.a. take care, is Python)
            sleep(0.1)


    def test_socket_status(self):
        """!Test the connection to the current address in field.
        @return bool with the socket status.
        """
        # Get the target server address
        host = self.entry.get()

        try:
            # Get the ip address and port
            ip_address, port = host.split(':')
        
            # Create the test socket
            test_socket = socket(AF_INET, SOCK_STREAM)

            test_socket.settimeout(0.5)

            # Attempt connection to the server to check status
            socket_alive = test_socket.connect_ex((ip_address, int(port)))

            rclpy.logging.get_logger("rosboard_gui_client").info(f"Port: {int(port)}")
            
            # Convert to boolean variable
            socket_alive = socket_alive == 0

            # Close socket connection
            test_socket.close()

            # Return the connection status
            return socket_alive

        except ValueError as e:
            return False

    def url(self):

        # Execute under try/except block to be decent
        try:
            # Get the IP address
            target_addr = self.entry.get()

            # Get the server IP address
            self.server_ip_addr = target_addr.split(":")[0]

            # Connect to rosboard client
            self.client = RosboardClient(host=target_addr, connection_timeout=5)
        
        # Handle the exception if connection is not possible
        except Exception as e:

            # Log the error message
            rclpy.logging.get_logger("rosboard_gui_client").error("Could not connect to server!")

            # Explicitly set the connection variable to false
            self.is_connected = False

            return

        # Set the variable to indicate that connection was achieved
        self.is_connected = True

        # 
        topics = self.client.get_available_topics()

        for button in self.buttons:
            button.destroy()
        self.buttons = []

        # Dictionary
        buttons = [{"pos": (n, 0), "text": topics[n]} for n in range(len(topics))]
        self.topic_widgets = {}

        def create_topic_spec(topic, n):

            i = self.SELECT_TOPICS // self.NUM_COLS
            j = self.SELECT_TOPICS % self.NUM_COLS
            self.SELECT_TOPICS += 1

            datainfospec = tk.Frame(self.datainfogen, background="light sky blue")
            datainfospec.grid(row=i, column=j, sticky="nsew", padx=5, pady=2)
            datainfospec.columnconfigure(0, weight=3)
            datainfospec.columnconfigure(1, weight=1)
            datainfospec.rowconfigure(0, weight=1)
            datainfospec.rowconfigure(1, weight=2)

            labelframe = LabelFrame(
                datainfospec, background="red", text=topic, borderwidth=5
            )
            labelframe.grid(row=0, column=0, columnspan=2, padx=5, pady=5)

            txt_network = tk.StringVar()
            txt_network.set("Rate [Hz]:  \n ΔT [ms]:")

            toplabel = Label(
                labelframe, textvariable=txt_network, height=2, width=15, font=16
            )
            toplabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5)

            button_close = tk.Button(
                datainfospec,
                text="X",
                command=partial(self.quit, topic, n),
                background="#8B0000",
            )
            button_close.grid(row=0, column=2, columnspan=1, sticky="e")

            self.topic_widgets[n] = [datainfospec]
            self.buttons[n].config(state="disabled")

            # Create the topic handler class
            topic_handler = TopicHandler(topic, self.client, self.node, txt_network, labelframe)

        for n, button in enumerate(buttons):
            self.buttons.append(
                tk.Button(
                    self.frame_buttons.container,
                    text=button["text"],
                    background="#40E0D0",
                    command=partial(create_topic_spec, button["text"], n),
                )
            )
            self.buttons[-1].grid(
                column=button["pos"][1],
                row=button["pos"][0],
                sticky="nsew",
                pady=5,
                padx=15,
            )

    def quit(self, topic, n):
        self.SELECT_TOPICS -= 1
        self.buttons[n].config(state="normal")

        # destroy the subscriptions
        self.client.destroy_socket_subscription(topic)

        for widget in self.topic_widgets[n]:
            widget.grid_remove()

        self.topic_widgets.pop(n)

        for n, value in enumerate(self.topic_widgets.values()):
            i = n // self.NUM_COLS
            j = n % self.NUM_COLS
            for widget in value:
                widget.grid_remove()
            for widget in value:
                widget.grid(row=i, column=j, padx=5, pady=5)


class TopicHandler:

    def __init__(self, topic, client, node, txt_network, frame):

        self.topic = topic

        # Define the topic type
        topic_type = client.get_topic_type(topic)

        # Create the republisher 
        # Get the republisher class for a given topic type
        republisher_class = PublisherManager.getDefaultPublisherForType(topic_type)

        # create the republisher object
        self.republisher = republisher_class(
            parent_node=node, topic_name=topic, topic_class_name=topic_type
        )
        # create the subscriptions with the republisher callback
        # each time a message arrives on a topic through rosboard
        # it will be published under the same topic name on the ros local system
        client.create_socket_subscription(
            topic_type, topic, self.topic_callback
        )

        # Store the text variable to change values
        self.txt_network = txt_network

        # Store the frame to change background color
        self.frame = frame

        # Attribute to store the initial time
        self.t_initial = time()

        # Store the received messages count
        self.n_msgs = 0

        # Define the number of time values used in the average calculation
        self.N = 20

        # Store the last message time
        self.t_last = None

        # List for storing the last N rate values
        self.t_last_window = []

        # Define the alpha for calculating the average
        self.alpha = 1.0

        # Calculate the required 1-alpha
        self.c_alpha = 1.0 - self.alpha

        # Attribute to handle the timer thread
        self.is_alive = True

        # Start the timeout thread
        timeout_th = Thread(target=self.timeout, daemon=True)
        timeout_th.start()

    def __del__(self):
        """! Kill the thread decently. """
        self.is_alive = False

    def timeout(self):
        while self.is_alive:
            if self.t_last is not None:
                if time() - self.t_last > 5.0:
                    self.frame.config(bg="gray")
            sleep(0.01)
        

    def calculate_average_rate(self, time_list):
        """! Calculate the average rate using EWMA.
            @param time_list: list with the last time delta values.
        """
        average = time_list[0]
        list_size = len(self.t_last_window)
        if list_size > 1:
            for indx in range(1, list_size):
                average = self.alpha * time_list[indx] + self.c_alpha * average
        return average

    def topic_callback(self, msg):

        # Increase the message count
        self.n_msgs += 1

        # Get the current time
        t_current = time()

        # Calculate average rate
        f_avg = self.n_msgs / (t_current - self.t_initial)

        # Discard first measurement
        if self.t_last is not None:

            # Update the window values
            self.t_last_window.insert(0, t_current - self.t_last)

            # Check if window list is full or not to discard values
            if len(self.t_last_window) > self.N:
                self.t_last_window.pop(-1)
            else:
                self.alpha = 2.0 / (len(self.t_last_window) + 1.0)
                self.c_alpha = 1.0 - self.alpha

            # Calculate the average using the window
            f_last = 1.0 / self.calculate_average_rate(self.t_last_window)

            # Define a red background if rate drops below 80% of average
            is_red = f_last < f_avg * 0.80

            # Update frame background conditionally
            if is_red:
                self.frame.config(bg="red")
            else:
                self.frame.config(bg="green")

            # Check if message includes header
            if "header" in msg[1].keys():

                # Get the message stamp
                stamp = msg[1]["header"]["stamp"]

                # Calculate the header timestamp
                t_send = stamp["sec"] + stamp["nanosec"] * 10 ** -9

                # Calculates the time delta
                t_delta = t_current - t_send

                # Set the network text label
                self.txt_network.set("Rate [Hz]: {:4.2f} \n ΔT [ms]: {:4.2f}".format(f_last, t_delta))
            
            # Defines t_delta as None if header not present
            else:
                
                # Set the network text label
                self.txt_network.set("Rate [Hz]: {:4.2f} \n ΔT [ms]: N/A".format(f_last))
            
        # Update last time value
        self.t_last = t_current

        # Parse and publish the message into ROS
        self.republisher.parse_and_publish(msg)


class RosboardGUINode(Node):
    def __init__(self):
        """!
        Node object that parses a yaml file to create rosboard subscriptions and
        republishing remote topics on the local machine
        """
        Node.__init__(self, node_name="rosboard_gui_client")
        self.logger = self.get_logger()


# =============================================================================
def main(args=None):
    """!
    Demonstration of the rosboard client using a yaml file to get topics
    """
    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the
    # executor is shutdown.
    rosboard_client = RosboardGUINode()

    # Runs callbacks in a pool of threads.
    executor = MultiThreadedExecutor()

    root = tk.Tk()
    app = Application(root, rosboard_client)
    root.geometry("1250x600")
    root.title("Kiwibot Features Interface")

    # Run root.mainloop in a separate python thread
    # thread = Thread(target=root.mainloop)
    # thread.daemon = True
    # thread.start()

    # Execute work and block until the context associated with the
    # executor is shutdown. Callbacks will be executed by the provided
    # executor.
    # rclpy.spin(rosboard_client, executor)

    # Spin node in a separate thread
    thread = Thread(target=rclpy.spin, args=(rosboard_client, executor))
    thread.daemon = True
    thread.start()

    root.mainloop()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rosboard_client.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()

# =============================================================================
