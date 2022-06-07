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

