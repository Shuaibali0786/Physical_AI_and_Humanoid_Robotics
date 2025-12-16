# Communication Patterns in ROS 2

ROS 2 provides several communication patterns for nodes to interact with each other. Understanding these patterns is crucial for designing effective robotic systems.

## Publish-Subscribe Pattern (Topics)

The publish-subscribe pattern is used for asynchronous, decoupled communication. Publishers send messages to topics without knowing who (if anyone) is subscribed. Subscribers receive messages from topics without knowing who published them.

### When to Use:
- Broadcasting sensor data
- Broadcasting robot state
- Event notifications
- Streaming data

### Advantages:
- Loose coupling between publishers and subscribers
- Multiple subscribers can receive the same data
- Asynchronous communication allows for non-blocking operations

### Disadvantages:
- No guarantee of message delivery
- No acknowledgment of receipt
- Potential for message loss if no subscribers exist

## Request-Response Pattern (Services)

The request-response pattern is used for synchronous, direct communication. A client sends a request to a service and waits for a response.

### When to Use:
- Configuration requests
- Action execution requests
- Data queries
- Operations that require confirmation

### Advantages:
- Synchronous: client waits for response
- Guaranteed delivery and processing
- Error handling through response status

### Disadvantages:
- Blocking: client must wait for response
- Only one server can handle a service request
- Less scalable for high-frequency operations

## Action Pattern

Actions are used for long-running tasks that require feedback and goal management. They combine the benefits of both topics and services.

### Components:
- **Goal**: Request sent to start an action
- **Feedback**: Periodic updates during action execution
- **Result**: Final outcome when action completes

### When to Use:
- Navigation tasks
- Complex manipulations
- Any long-running task requiring monitoring

## Quality of Service (QoS) Settings

QoS settings allow fine-tuning of communication behavior to meet specific requirements.

### Key QoS Policies:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local (for late-joining subscribers)
- **History**: Keep all messages vs. keep last N messages
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to determine if a publisher is alive

## Best Practices

1. **Use topics for streaming data** like sensor readings or robot states
2. **Use services for configuration and queries** that require immediate responses
3. **Use actions for long-running tasks** that need feedback
4. **Configure appropriate QoS settings** based on your application's requirements
5. **Design message types carefully** to optimize bandwidth and processing