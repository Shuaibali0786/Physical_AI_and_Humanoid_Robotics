# Exercises: ROS 2 Basics

## Exercise 1: Understanding Nodes, Topics, and Services

### Objective
Demonstrate understanding of fundamental ROS 2 concepts.

### Instructions
1. Create a simple ROS 2 workspace
2. Create a publisher node that publishes "Hello, World!" messages to a topic called "chatter"
3. Create a subscriber node that listens to the "chatter" topic and logs the received messages
4. Run both nodes and verify that messages are properly exchanged

### Expected Outcome
- Publisher node successfully sends messages
- Subscriber node successfully receives and logs messages
- Clear understanding of the publish-subscribe pattern

## Exercise 2: Service Implementation

### Objective
Implement a simple request-response service.

### Instructions
1. Create a service server that accepts two integer numbers and returns their sum
2. Create a service client that sends two numbers to the server
3. Verify that the client receives the correct sum from the server

### Expected Outcome
- Service server correctly processes requests
- Service client receives correct responses
- Understanding of synchronous communication

## Exercise 3: Quality of Service Settings

### Objective
Explore different QoS settings and their effects.

### Instructions
1. Create a publisher with reliable delivery QoS
2. Create a publisher with best-effort delivery QoS
3. Compare the behavior of both publishers with the same subscriber
4. Document the differences in message delivery

### Expected Outcome
- Understanding of how QoS settings affect communication
- Ability to choose appropriate QoS settings for different use cases

## Self-Assessment Questions

1. What is the difference between a node, topic, and service in ROS 2?
2. When would you use publish-subscribe vs. request-response communication?
3. What are Quality of Service (QoS) settings and why are they important?
4. How does ROS 2 handle multiple publishers and subscribers on the same topic?