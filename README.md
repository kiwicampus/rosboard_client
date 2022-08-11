# Rosboard Desktop Client

The main objective of this library is to selectively stream ROS topics using websockets from a remote robot to a local machine, allowing to use visualization tools like rviz, rqt, mapviz, etc on live data transmitted over the internet. The library implements a client to use with a [rosboard](https://github.com/kiwicampus/rosboard) server running in the remote robot.

## Existing alternatives
There are already some alternatives that allow to transmit ros data over the internet, however they have some drawbacks 

- Connecting a local ROS2 system to a remote one using a VPN like [Husarnet](https://husarnet.com/docs/tutorial-ros2). This removes the possibility of selecting only some topics for transmission, and does not perform any compression of the data, which makes it unsuitable for robots running high data rate sensors like cameras and LIDAR and relying on a cellular connection to stream the data.

- Using [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite). Though this does allow to stream topics selectively through websockets, its compression proved inefficient for large messages like images and occupancy grids, which lead to a very high bandwidth usage, lag "accumulation" and a severe drop in the rate of messages arriving to the desktop client.

Rosboard showed to overcome this drawbacks, keeping a comparatively lower bandwidth usage when streaming data intensive messages and having lower non-accumulating lag.

## Brief functionality description
This library provides a desktop client that connects to a remote [rosboard](https://github.com/kiwicampus/rosboard) server and allows to create subscriptions to topics on that server. When a message is received binary data is decoded and passed to a user specified callback.

The library also provides a set of _republishers_ meant to convert json data from rosboard back to ROS messages and republish them on the local machine. They account for the special subsampling and compression made by rosboard on Images, LaserScans, PointCloud2s and OccupancyGrids, and can convert virtually any other type of messages by relying on the [`rclpy_message_converter`](rospy_message_converter/rclpy_message_converter/) library, added as a submodule in this package.

The networking module is decoupled from the ROS dependent republishers, so it can be used as a standalone package providing custom callbacks for subscriptions that handle the json payloads. The socket client decompresses and decodes all the data before passing it to the callback.

An easy way of running this over the internet is using [`ngrok`](https://ngrok.com/). After setting your authtoken you can tunnel the rosboard server, usually in the 8888 port using the tcp protocol: 
```
ngrok tcp 8888
```
The url that ngrok provides can be used as host for creating the rosboard client

## Rosboard GUI
This package includes a graphical user interface (GUI) that can be used to connect the local computer (client) to the server. The GUI allows an user to define which topics will be streamed from the server to the client and viceversa. In addition to such capability, the interface presents information related to the rate in which the topic messages are received and the latency of them i.e. the time difference between the message header stamp and the current system time. Finally, some general metrics are presented regarding the CPU usage, average roundtrip time (RTT), and current download speed. This application is intended to ease the streaming process while being capable of dynamically selecting which topics are streamed.

<p align="center"><img src="https://user-images.githubusercontent.com/14006555/183146092-0c4fd61f-8222-4fdb-baa7-5f38cecb30a2.png" alt="rosboard-gui" width=800></p>


## Comparison with Rosbridge
We think of *rosbridge* as the main alternative to *rosboard*. In this context, we want to prove that *rosboard* presents an improved approach to streaming data from a remote device within the context of ROS. To do so, we propose a benchmark of metrics relevant to the streaming process. We must note that we will provide any third party with the capability of executing their own tests. We will benchmark two (2) metrics: latency and download speed.

- *Latency:* we define latency as the time difference between the timestamp of the received message and the current computer time. We must note that there might be difference in the server and client clock that might affect this measurement.
- *Bandwidth Usage*: is the measurement of received bytes per second while using rosboard. Keep in mind that this measurement is a system-wide metric. In this context, any additional process using the network interface might contaminate the test.

Two laptops were configured in a Local Area Network (LAN) using a network switch. The network switch was not connected to the internet in order to prevent contaminating the test. The used switch had a bandwidth limit of 100 Mibps. We did the benchmark with a data-intensive topics: a RGB camera streaming. The next table presents the used specifications for the streamed topic:

<table align="center">
    <tr>
        <th>Topic</th>
        <th>Resolution</th>
        <th>FPS</th>
    </tr>
    <tr>
        <td>RGB Camera</th>
        <td>360 x 640</th>
        <td>15</th>
    </tr>
</table>

We did our tests streaming the camera feed from a computer connected to a LAN.

<table align="center">
    <tr>
        <th>Streaming Tool</th>
        <th>Avg. Latency [ms]</th>
        <th>Avg. Bandwidth Usage [KiB/s]</th>
    </tr>
    <tr>
        <td>Rosboard (JPEG compression)</td>
        <td align="right">96.38</td>
        <td align="right">11010.14</td>
    </tr>
    <tr>
        <td>Rosbridge (no compression)</td>
        <td align="right">22330.06</td>
        <td align="right">12420.32</td>
    </tr>
    <tr>
        <td>Rosbridge (CBOR compression)</td>
        <td align="right">6035.99</td>
        <td align="right">8215.36</td>
    </tr>
    <tr>
        <td>Rosbridge (PNG compression)</td>
        <td align="right">19096.12</td>
        <td align="right">12054.46</td>
    </tr>
</table>

A visual representation on the difference between `rosboard` and `rosbridge` can be seen in the following GIFs. These GIFs were recorded in the client side while streaming the color image. The streaming visualized using Rviz and captured by recording the screen. This visualization and recording took place in the client ROS network. You may notice how `rosboard` (left) has less latency than `rosbridge` (right, no compression).

<table align="center">
    <tr>
        <td><p align="center"><img src="https://user-images.githubusercontent.com/14006555/184149174-0b270bcf-7f86-4dfa-aec5-be5d608bed9f.gif" alt="rosboard-chrono" height=200></p></td>
        <td><p align="center"><img src="https://user-images.githubusercontent.com/14006555/184149101-bad405e1-84b2-4f6e-8cbe-991be1ddd421.gif" alt="rosbridge-chrono" height=200></p></td>
    </tr>
</table>

### Running your own benchmarks

To run the `rosboard` benchmarking, you need to follow the next steps in the server side:
1. Run the [rosboard](https://github.com/kiwicampus/rosboard) node by running `ros2 run rosboard rosboard_node`. You may use the server IP address to connect from the client or forward the port using a third-party tool such as [Ngrok](https://ngrok.com/).

Run the next steps in the client side:
1. Configure the [`rosboard_config.yaml`](./rosboard_desktop_client/rosboard_desktop_client/benchmarks/rosboard_config.yaml) file with the URL of the server and the topics you want to stream:
    - To configure the URL, set the `url` value in the file.
    - To configure the topics you want to stream *from* the server, configure the topics in the `topics` value in the file.
    - If you want to stream topics *to* the server, configure the `topics_to_stream` value in the file.
2. Run the `rosboard` benchmark script by executing `ros2 run rosboard_desktop_client rosboard_benchmark`. Let the script run for at least a minute and for it to print the test results in console. You may now exit the script.

To run the `rosbridge` benchmarking, you need to follow the next steps in the server side:
1. Run the `ros2 launch rosbridge_server rosbridge_websocket.py`.

Run the next steps in the client side:
1. Configure the [`rosbridge_config.yaml`](./rosboard_desktop_client/rosboard_desktop_client/benchmarks/rosbridge_config.yaml) file with the topics you want to stream:
    - Configure the URL in the corresponding field in the YAML file.
    - Add a topic by setting it as a key in the YAML file for the `topics_to_subscribe` entry.
    - Set the message type in the `type` field.
    - Set the compression algorithm in the `compression` field.
2. Run the `rosbridge` benchmark script by executing `ros2 run rosboard_desktop_client rosboard_benchmark`. Let the script run for at least a minute and for it to print the test results in console. You may now exit the script.

**Note:** to run these tests, we used our own `rosboard`, `rosbridge` and `roslibpy` forks. In the case of `rosboard`, our forks allow us to fix an issue regarding ROS2 QoS ([PR](https://github.com/dheera/rosboard/pull/104)) and adding the feature of streaming topics from the client to the server. With regard of `rosbridge`, these forks correct some issues that are present in the main `rosbridge` repository such as allowing the CBOR and PNG compression for images when using ROS2. We strongly recommend using such forks to run the test. The links to our forks are:
- [`rosboard`](https://github.com/kiwicampus/rosboard): our own `rosboard` fork.
- [`rosbridge_suite`](https://github.com/kiwicampus/rosbridge_suite): use the `cbor_fix` branch when compiling the package.
- [`roslibpy`](https://github.com/kiwicampus/roslibpy): use the `cbor` branch for CBOR compression and `cbor_png` for PNG compression.


----

### ROS independent usage
```.py
from rosboard_desktop_client.networking import RosboardClient

def cb(rosboard_data):
    print(f"got data: {rosboard_data}")

# this will create a client that is independent from ROS
client = RosboardClient(host='localhost:8888', connection_timeout=5)

topic = "/fix"

# Upon being created, the client stores the available topics in the remote server
# This raises an exception if the topic is not available
topic_type = client.get_topic_type(topic)

# Creates a subscription and runs the callback each time data is received
client.create_socket_subscription(msg_type=topic_type, topic=topic, callback=cb)

# The rest of your code. The client runs on a separate thread

```

### Minimal ROS republisher node
```.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rosboard_desktop_client.republishers import PublisherManager
from rosboard_desktop_client.networking import RosboardClient


class RosboardClientNode(Node):
    def __init__(self):
        Node.__init__(self, node_name="rosboard_client")

        topics = ["/scan", "/tf", "/fix"]

        client = RosboardClient(host="localhost:8888", connection_timeout=5)

        for topic in topics:
            # Check if topic is available
            if not client.is_topic_available(topic):
                continue

            topic_type = client.get_topic_type(topic)

            # Get the republisher class for a given topic type
            republisher_class = PublisherManager.getDefaultPublisherForType(topic_type)

            # create the republisher object
            republisher = republisher_class(
                parent_node=self, topic_name=topic, topic_class_name=topic_type
            )

            # create the subscriptions with the republisher callback
            # each time a message arrives on a topic through rosboard
            # it will be published under the same topic name on the ros local system
            client.create_socket_subscription(
                topic_type, topic, republisher.parse_and_publish
            )


def main(args=None):
    # The usual ROS2 stuff
    rclpy.init(args=args)

    rosboard_client = RosboardClientNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(rosboard_client, executor)

    rosboard_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

