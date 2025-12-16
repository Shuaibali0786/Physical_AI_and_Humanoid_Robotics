# ROS 2 Nodes, Topics, and Services

## Nodes

A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Each node is designed to perform a specific task and communicates with other nodes through topics, services, and actions.

### Creating a Node

In ROS 2, nodes are typically created using client libraries like `rclpy` for Python or `rclcpp` for C++. Each node must have a unique name within its namespace.

Key characteristics of nodes:
- Each node runs in its own process
- Nodes can be written in different programming languages
- Nodes can be started and stopped independently
- Nodes must be part of a ROS 2 graph to communicate

## Topics and Message Passing

Topics enable asynchronous communication between nodes through a publish-subscribe model. Publishers send messages to a topic, and subscribers receive messages from the topic.

### Key Concepts:
- **Publishers**: Nodes that send data to a topic
- **Subscribers**: Nodes that receive data from a topic
- **Messages**: Data structures that are passed between nodes
- **Quality of Service (QoS)**: Settings that define communication behavior

### Communication Pattern:
1. A publisher node sends messages to a named topic
2. The ROS 2 middleware routes messages to all subscribers of that topic
3. Subscribers receive messages asynchronously

## Services

Services provide synchronous request-response communication between nodes. A client sends a request to a service, and the service sends back a response.

### Key Concepts:
- **Service Server**: Node that provides a service
- **Service Client**: Node that uses a service
- **Service Interface**: Defines the request and response message types
- **Synchronous**: The client waits for the response before continuing

### Communication Pattern:
1. A client sends a request to a service
2. The service processes the request
3. The service sends back a response
4. The client receives the response and continues

## Practical Example: Publisher-Subscriber

In the next section, we'll implement a simple publisher-subscriber example that demonstrates these concepts in practice.