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
from psutil import cpu_percent
from time import sleep
import tkinter as tk
from functools import partial
from pathlib import Path
from threading import Thread
from tkinter import *

import rclpy
import yaml
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

        self.contenedor = tk.Frame(self.canvas)
        self.contenedor_id = self.canvas.create_window(
            0, 0, window=self.contenedor, anchor=tk.NW
        )
        self.contenedor.bind("<Configure>", self._configure_contenedor)
        self.canvas.bind("<Configure>", self._configure_canvas)

    def _configure_contenedor(self, event):
        size = (self.contenedor.winfo_reqwidth(), self.contenedor.winfo_reqheight())
        self.canvas.config(scrollregion="0 0 {} {}".format(*size))
        if self.contenedor.winfo_reqwidth() != self.canvas.winfo_width():
            self.canvas.config(width=self.contenedor.winfo_reqwidth())
        if self.contenedor.winfo_reqheight() != self.canvas.winfo_height():
            self.canvas.config(height=self.contenedor.winfo_reqheight())

    def _configure_canvas(self, event):
        if self.contenedor.winfo_reqwidth() != self.canvas.winfo_width():
            self.canvas.itemconfigure(
                self.contenedor_id, width=self.canvas.winfo_width()
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
        self.entry.insert(END, "localhost:8888")
        self.entry.grid(row=1, column=2, columnspan=3, sticky=tk.W + tk.E, padx=5)

        buttonCon = tk.Button(
            self.frame_conf,
            text="CONNECT",
            command=self.url,
            background="light sky blue",
        )
        buttonCon.grid(row=1, column=5, columnspan=2, sticky="ew")

        ButtonStat = tk.Button(
            self.frame_conf, text="STATUS", bg="#90EE90", state=DISABLED
        )
        # Light green #90EE90
        # Grey #D3D3D3
        # Light red #FF7F7F
        ButtonStat.grid(row=1, column=9, columnspan=1, sticky="ew")

        # CPUData=0
        # RoundtripData=0
        # DownloadData=0


        # ---------------------------------------------------------------
        # CPU Label
        # ---------------------------------------------------------------
        # Create the string variable to update text
        self.cpu_label_txt = tk.StringVar()
        self.cpu_label_txt.set("CPU [%]: {:4.2f}".format(0.0))

        # Define the CPU usage label
        cpu_usage_lb = tk.Label(self.frame_conf, textvariable=self.cpu_label_txt, background="light sky blue")
        cpu_usage_lb.grid(row=0, column=10, columnspan=2, sticky="w", padx=5, pady=2)
        # ---------------------------------------------------------------

        # ---------------------------------------------------------------
        # Roundtrip label
        # ---------------------------------------------------------------
        # Create the string variable to update text
        self.rt_label_txt = tk.StringVar()
        self.rt_label_txt.set("Roundtrip(ms): {:5.2f}".format(0.0))

        # Define the roundtrip label (from ping-ing the server IP address)
        roundtrip_lb = tk.Label(
            self.frame_conf, text="", background="light sky blue"
        )
        roundtrip_lb.grid(row=1, column=10, columnspan=2, sticky="w", padx=5, pady=2)
        # ---------------------------------------------------------------

        label4 = tk.Label(
            self.frame_conf, text="Download(MB/s)", background="light sky blue"
        )
        label4.grid(row=2, column=10, columnspan=2, sticky="w", padx=5, pady=2)

        self.frame_stats = tk.Frame(self.master, background="light sky blue")
        self.frame_stats.pack()

        self.frame_buttons = ScrolledFrame(self.frame_stats)
        self.frame_buttons.grid(row=1, column=1, columnspan=2)

        self.datainfogen = tk.Frame(self.frame_stats, background="light sky blue")
        self.datainfogen.grid(
            row=1, column=3, columnspan=5, sticky="nsew", padx=5, pady=2
        )

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
        while self.is_gui_running:

            # Get the process CPU utilization
            cpu_percent_val = cpu_percent()

            # Update the CPU utilization text variable
            self.cpu_label_txt.set("CPU [%]: {:4.2f}".format(cpu_percent_val))

            # Sleep to avoid being inefficient (a.k.a. take care, is Python)
            sleep(0.1)


    def url(self):
        host = self.entry.get()
        self.client = RosboardClient(host=host, connection_timeout=5)
        topics = self.client.get_available_topics()

        for button in self.buttons:
            button.destroy()
        self.buttons = []

        # Dictionary
        buttons = [{"pos": (n, 0), "text": topics[n]} for n in range(len(topics))]
        self.topic_widgets = {}

        def create_topic_spec(topic, n):
            conect = 0
            if conect == 1:
                bg = "green"
            elif conect == 0:
                bg = "red"

            i = self.SELECT_TOPICS // self.NUM_COLS
            j = self.SELECT_TOPICS % self.NUM_COLS
            self.SELECT_TOPICS += 1

            datainfospec = tk.Frame(self.datainfogen, background="light sky blue")
            datainfospec.grid(row=i, column=j, sticky="nsew", padx=5, pady=2)

            labelframe = LabelFrame(
                datainfospec, background=bg, text=topic, borderwidth=5
            )
            labelframe.grid(row=0, column=0, columnspan=2, padx=5, pady=5)

            toplabel = Label(
                labelframe, text="Rate (Hz):\nΔT (ms):", height=2, width=15, font=16
            )
            toplabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5)

            button_close = tk.Button(
                datainfospec,
                text="X",
                command=partial(self.quit, topic, n),
                background="#8B0000",
            )
            button_close.grid(row=0, column=2, padx=5, pady=5)

            self.topic_widgets[n] = [datainfospec]
            self.buttons[n].config(state="disabled")

            topic_type = self.client.get_topic_type(topic)
            # Get the republisher class for a given topic type
            republisher_class = PublisherManager.getDefaultPublisherForType(topic_type)

            # create the republisher object
            republisher = republisher_class(
                parent_node=self.node, topic_name=topic, topic_class_name=topic_type
            )
            # create the subscriptions with the republisher callback
            # each time a message arrives on a topic through rosboard
            # it will be published under the same topic name on the ros local system
            self.client.create_socket_subscription(
                topic_type, topic, republisher.parse_and_publish
            )

        for n, button in enumerate(buttons):
            self.buttons.append(
                tk.Button(
                    self.frame_buttons.contenedor,
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
                padx=1,
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
    root.geometry("1200x700")
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
