# Exercises: Python Agents & ROS 2 Integration

## Exercise 1: Basic rclpy Node

### Objective
Create a simple Python node that publishes messages to a topic.

### Instructions
1. Create a new Python file called `simple_publisher.py`
2. Implement a node that publishes "Hello from Python" messages to a topic called "python_chatter"
3. Use rclpy to create the publisher and node
4. Run the node and verify it's publishing messages

### Expected Outcome
- Node successfully created using rclpy
- Messages published to the topic at regular intervals
- Understanding of basic rclpy structure

## Exercise 2: Publisher-Subscriber Pair in Python

### Objective
Create a publisher and subscriber in Python that communicate with each other.

### Instructions
1. Create a publisher node that sends integer messages
2. Create a subscriber node that receives and logs these messages
3. Use custom message types if desired
4. Run both nodes and verify communication

### Expected Outcome
- Publisher sends messages successfully
- Subscriber receives and processes messages
- Understanding of Python pub-sub pattern

## Exercise 3: Python Service Server and Client

### Objective
Implement a service server and client in Python.

### Instructions
1. Create a service server that performs a calculation (e.g., calculate factorial)
2. Create a client that sends requests to the server
3. Test the request-response communication
4. Verify the client receives correct responses

### Expected Outcome
- Service server processes requests correctly
- Client receives expected responses
- Understanding of Python service implementation

## Exercise 4: Simple Python Agent

### Objective
Create a Python agent that responds to sensor data.

### Instructions
1. Create an agent that subscribes to a sensor topic (e.g., simulated laser scan)
2. Implement simple decision logic (e.g., stop if obstacle is too close)
3. Have the agent publish movement commands based on sensor input
4. Test the agent's behavior with simulated sensor data

### Expected Outcome
- Agent responds appropriately to sensor input
- Clear understanding of agent behavior patterns
- Basic robot control through ROS 2

## Exercise 5: Parameter Management

### Objective
Use parameters to configure a Python node.

### Instructions
1. Create a node that uses parameters for configuration
2. Declare and use at least 3 different parameter types
3. Test changing parameters at runtime
4. Implement parameter validation

### Expected Outcome
- Node properly handles parameters
- Parameters can be changed at runtime
- Understanding of parameter management in Python

## Self-Assessment Questions

1. How do you create a node using rclpy?
2. What is the difference between a publisher and subscriber in Python?
3. How do you implement a service server in Python?
4. What are the advantages of using Python for ROS 2 agents?
5. How do you handle parameters in rclpy nodes?