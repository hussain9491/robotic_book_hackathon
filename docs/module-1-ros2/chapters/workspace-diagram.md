# ROS 2 Workspace Diagram

## Directory Structure Diagram

```
~/ros2_ws/ (workspace root)
├── src/ (source code)
│   ├── hello_robot_pkg/
│   │   ├── hello_robot_publisher.py
│   │   └── hello_robot_subscriber.py
│   ├── utilities.py
│   ├── config_manager.py
│   └── (other packages...)
├── build/ (build artifacts)
│   ├── hello_robot_pkg/
│   └── (build files...)
├── install/ (installed packages)
│   ├── lib/
│   ├── share/
│   └── (installed files...)
└── log/ (log files)
    ├── latest/
    └── (log files...)
```

## Node Communication Diagram

```mermaid
graph LR
    A[hello_robot_publisher] -->|publishes| B(hello_robot topic)
    B -->|subscribes| C[hello_robot_subscriber]

    style A fill:#e1f5fe
    style C fill:#e8f5e8
    style B fill:#fff3e0
```

## System Architecture Diagram

```mermaid
graph TB
    subgraph "Robot System"
        A[Humanoid Robot]
    end

    subgraph "ROS 2 System"
        B[ROS 2 Middleware]
        C[Node: Joint Controllers]
        D[Node: Sensor Processors]
        E[Node: Path Planner]
        F[Node: hello_robot_publisher]
        G[Node: hello_robot_subscriber]
    end

    subgraph "Development Environment"
        H[Development Machine]
        I[RViz2 Visualizer]
        J[Command Line Tools]
    end

    B --> C
    B --> D
    B --> E
    B --> F
    B --> G
    C <--> D
    E --> F
    G --> I
    J --> B

    style B fill:#e3f2fd
    style H fill:#f3e5f5
    style I fill:#e8f5e8
    style J fill:#fff3e0
```

## Communication Flow Diagram

```mermaid
sequenceDiagram
    participant P as Publisher Node
    participant DDS as DDS Middleware
    participant S as Subscriber Node

    P->>DDS: Publish "Hello Robot" message
    DDS->>S: Deliver message
    S->>S: Process received message
```

## Package Organization

The workspace is organized with a modular approach:

- **Core Packages**: Basic robot functionality (communication, basic control)
- **Sensor Packages**: Sensor data processing and integration
- **Control Packages**: Joint controllers and motion planning
- **Utility Packages**: Common utilities and configuration management
- **Demo Packages**: Examples and tutorials (like hello_robot_pkg)

This structure allows for scalable development where each component can be developed, tested, and maintained independently while maintaining clear communication interfaces through ROS 2 topics, services, and actions.