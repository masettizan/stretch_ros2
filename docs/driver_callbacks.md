# Executors and Callback Groups in Stretch's ROS2 Driver

When writing a ROS2 program, you can control execution flow by using executors and callback groups. In a ROS2 node with lots of callbacks, such as Stretch's ROS2 driver, it's important to carefully select executors and callback groups to ensure callbacks are running in the most efficient way possible, and that no deadlocks are possible. In this document, we capture how the driver's execution and callback groups are currently set up, and provide insight into the design decision behind why it's set up this way.

## Background

We'll need to know about:

 1. ROS2 callbacks, executors, and callback groups
 2. The driver's callbacks 

### ROS2 Callbacks, Executors, and Callback Groups

Callbacks are functions that get called when an event happens. For example, when subscribing to a ROS2 topic, we write a callback function that gets called when the topic gets a message. There's a few types of callbacks:

 - subscription callbacks (for handling the message received from a topic)
 - service callbacks (for when our ROS2 service receives a request)
 - timer callbacks (called at a regular rate)
 - action server callbacks (for starting/cancelling/etc. a "goal" send by a client)
 - action client callbacks (for results from the action server for our starting/cancelling/etc. the goal)
 - futures (for other times the client has to wait for a result)

Rclpy offers two different executors:

 - SingleThreadedExecutor (default)
 - MultiThreadedExecutor

The single threaded executor takes every callback in the program, and executes it serially in a single thread. For the driver, which has a lot of callbacks, this type of executor will cause lag if the driver receives a lot of requests. The multi-threaded executor uses multiple threads to execute callbacks in parallel.

Rclpy offers two different kinds of callback groups:

 - MutuallyExclusiveCallbackGroup (default)
 - ReentrantCallbackGroup

TODO

### The Driver's Callbacks

TODO
