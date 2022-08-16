<img src="https://user-images.githubusercontent.com/43115782/167684386-91cc219e-c499-4f5d-8895-2b292f887563.png" alt="kiwi_banner" width="1200">

# Rosboard Client
This package is a graphical user interface (GUI) to be used with the rosboard utility. The graphical interface allows an user to subscribe to topics that are available in the server and stream topics to the server. By using an user interface it is possible to dynamically configure which topics will be streamed to and from the server. This guide will introduce the interface along with its functionalities. The GUI is presented in the next image. This shows how the GUI would look under normal operations. The interface is composed of four main widgets: the connection widget, the statistics widget, the server-available widget and the client-available widget.

<p align="center"><img src="https://user-images.githubusercontent.com/14006555/184432712-2b108dcc-fff3-4ec1-b229-d6f71c3dc1f6.png" alt="rosboard-gui" width=800></p>

## Connection Widget
The connection widget allows an user to manage the connection from the GUI to the server. It is expected for the user to input the server address in the line editor and then click the "CONNECT" button to establish the connection to the server. A feedback is presented in the widget and it shows the connection status. Such status can have three status: "CONNECTED", "DISCONNECTED" and "RETRYING". The "RETYING" status is used when the connection to the server is lost and the interface is attempting to regain such connection. Clicking the "DISCONNECT" button will close the connection to the server.

<p align="center"><img src="https://user-images.githubusercontent.com/14006555/183149084-e45960d2-3819-4bf9-b64a-50fa36ab964b.png" alt="connection-widget" width=800></p>

## Statistics Widget
It show three statistics regarding the user interface. Three statistics are shown: CPU usage percentage, roundtrip time between interface and server in milliseconds, and download speed of host computer. The CPU usage and download speed are continuously updated even if the interface is not connected to the server. The roundtrip time is updated only if the GUI is connected to the server.

<p align="center"><img src="https://user-images.githubusercontent.com/14006555/183149252-9cfc4a98-8478-4f7d-b445-66112dc6999d.png" alt="statistics-widget" width=800></p>

## Server-available Widget
It shows a list and a panel with the topics that are available in the server to be streamed to the client. If an element from the list (left side) is clicked, the streaming will start and the topic will be shown in the panel (right side). Topics in the panel will be accompanied of its measured publish frequency and latency. The frequency is measured from the time at which the messages are received in the client and the latency is the time difference between time-stamped messages and the time at which they were received. Closing a topic element in the panel will stop it from being streamed.

<p align="center"><img src="https://user-images.githubusercontent.com/14006555/183149517-af6dda89-3a06-477d-a56e-121e562e9922.png" alt="server-widget" width=800></p>

## Client-available Widget
It also shows a list and a panel, however, it shows the topics that are available in the client. If an element of the list is clicked, it will be streamed to the server. If the "X" in the topic widget is pressed, the topic will cease to be streamed.

<p align="center"><img src="https://user-images.githubusercontent.com/14006555/183149590-f4fcc299-519d-448f-a248-39a514bd4716.png" alt="client-widget" width=800></p>
