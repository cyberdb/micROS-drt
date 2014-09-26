micROS-RT
=========

micROS RT (micROS Real-Time) is a modified ROS C++ kernel which adopts OMG's DDS (Data Distribution Service for Real-time Systems) as its underlying message transfer protocol. DDS is an Object Management Group's standard for pub/sub middleware (http://portals.omg.org/dds/). It supports high-performance, scalable and QoS-assuring message delivery. It has been applied into many industry-level systems. By replacing the original ROS message protocols (TCPROS & UDPROS) with DDS, a set of advanced features can be supported in the ROS message delivery process.

(1) **Built-in multicast support**. When there are _n_ listeners in a topic (_n_>=2), significant performance advantage can be obtained.

(2) **Robustness in some adverse network environments**. For example, it has better reconnection behavior when dropping out of wireless (according to the report of Dirk Thomas [link](http://roscon.ros.org/2014/wp-content/uploads/2014/07/ROSCON-2014-Next-Generation-of-ROS-on-top-of-DDS.pdf)).

(3) **Real-time and other QoS assurance in message delivery**. For example, you can set the transport priority and latency budget of messages, specify expected message arriving deadline and the behavior when the deadline is not met, set time-based filter to the messages on a topic, and so on.

Existing ROS packages can easily benefit from feature 1 and 2. No modification/recompilation is needed and the only thing you should do is to replace a library file in the ROS installation path. If you want to specify the real-time and other QoS properties (feature 3), you can use two newly added APIs in your program(advertiseWithQoS() and subscribeWithQoS()). More details can be found in the user's manual of micROS RT ([download](https://github.com/cyberdb/micROS-RT/blob/master/docs/manual.pdf)).


Release Notes
----------------
v0.20(2014-09-25)

1) Adding QoS setting APIs

2) Adding latch support

3) Minor bug fixes

Known issues

1) Callback functions to get the notification of subscriber status changes (connected and disconnected) will not be invoked.
