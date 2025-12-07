# ROS 2 Architecture

## Distributed Architecture

ROS 2 follows a distributed architecture where different processes (nodes) communicate through topics, services, and actions. This design allows for modularity and scalability.

## Core Architecture Components

### 1. Nodes
**Nodes** are individual processes that perform specific tasks. Each node:
- Runs independently
- Can be started/stopped separately
- Communicates with other nodes via topics, services, or actions
- Can be written in Python, C++, or other languages

### 2. Topics
**Topics** are asynchronous communication channels for streaming data:
- **Publisher**: Sends data to a topic
- **Subscriber**: Receives data from a topic
- **One-to-many**: Multiple subscribers can receive from one publisher
- **Decoupled**: Publishers and subscribers don't need to know about each other

### 3. Services
**Services** provide synchronous request-response communication:
- **Client**: Makes a request
- **Server**: Processes the request and sends a response
- **Blocking**: Client waits for response
- **One-to-one**: Direct communication between client and server

### 4. Actions
**Actions** handle long-running tasks with feedback:
- **Client**: Sends a goal
- **Server**: Executes the task and sends feedback
- **Non-blocking**: Client can continue other work
- **Feedback**: Periodic updates on task progress

## Communication Patterns

### Publisher-Subscriber Pattern
```
Node A (Publisher) → Topic → Node B (Subscriber)
                   → Topic → Node C (Subscriber)
```

### Client-Server Pattern
```
Node A (Client) → Service Request → Node B (Server)
                ← Service Response ←
```

### Action Pattern
```
Node A (Client) → Goal → Node B (Action Server)
                ← Feedback ←
                ← Result ←
```

## DDS (Data Distribution Service)

ROS 2 uses DDS as its underlying communication middleware:
- **Discovery**: Automatic discovery of nodes and topics
- **Quality of Service (QoS)**: Configurable reliability and durability
- **Security**: Built-in encryption and authentication
- **Performance**: Optimized for real-time systems

## ROS 2 Graph

The ROS 2 graph represents all nodes, topics, services, and actions in the system:
- **ros2 node list**: List all running nodes
- **ros2 topic list**: List all active topics
- **ros2 service list**: List all available services
- **ros2 action list**: List all available actions

## Best Practices

1. **Modularity**: Keep nodes focused on single responsibilities
2. **Naming**: Use descriptive names for nodes, topics, and services
3. **Namespaces**: Organize related nodes under namespaces
4. **Parameters**: Use parameters for configuration instead of hardcoding
5. **Error Handling**: Implement robust error handling in all nodes

