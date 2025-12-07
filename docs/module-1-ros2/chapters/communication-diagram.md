# Communication Diagram: Humanoid Robot Control Cycle

## Overview

This diagram illustrates the communication patterns in a humanoid robot control system, showing how different components interact through ROS 2 topics, services, and parameters.

## System Architecture

```mermaid
graph TB
    subgraph "Humanoid Robot"
        A[Physical Robot]
        B[Actuators]
        C[Sensors]
    end

    subgraph "ROS 2 Middleware"
        DDS[DDS - Data Distribution Service]
    end

    subgraph "Control System Nodes"
        D[Sensor Processing Node]
        E[State Estimation Node]
        F[Path Planning Node]
        G[Controller Node]
        H[Joint Control Node]
        I[High-level Planner]
    end

    subgraph "External Systems"
        J[RViz Visualization]
        K[Command Interface]
        L[Logging System]
    end

    A --> B
    A --> C
    C --> D
    D --> DDS
    DDS --> E
    E --> F
    F --> G
    G --> H
    H --> B
    DDS --> J
    K --> DDS
    DDS --> L

    style A fill:#e1f5fe
    style DDS fill:#f3e5f5
    style J fill:#e8f5e8
    style K fill:#fff3e0
    style L fill:#fce4ec
</>

## Communication Flow

```mermaid
sequenceDiagram
    participant S as Sensors
    participant SP as Sensor Processing
    participant SE as State Estimation
    participant PP as Path Planning
    participant C as Controller
    participant JC as Joint Control
    participant A as Actuators

    S->>SP: Raw sensor data (topics)
    SP->>SE: Processed sensor data (topics)
    SE->>PP: Robot state (topics)
    PP->>C: Desired trajectory (topics)
    C->>JC: Control commands (topics)
    JC->>A: Actuator commands
    A-->>S: Feedback loop
```

## Topic Communication Pattern

```mermaid
graph LR
    subgraph "Publisher Nodes"
        A[Joint State Publisher]
        B[IMU Data Publisher]
        C[Camera Publisher]
    end

    subgraph "ROS 2 Topics"
        T1[joint_states]
        T2[imu/data]
        T3[camera/image_raw]
    end

    subgraph "Subscriber Nodes"
        D[Controller Node]
        E[State Estimator]
        F[Perception Node]
    end

    A --> T1
    B --> T2
    C --> T3
    T1 --> D
    T2 --> E
    T3 --> F

    style T1 fill:#e3f2fd
    style T2 fill:#e8f5e8
    style T3 fill:#fff3e0
```

## Service Communication Pattern

```mermaid
graph LR
    A[Planning Node]
    B[Service Server<br/>Joint Control]
    C[Service Client<br/>Motion Request]

    A -.-> C
    C -.-> B
    B -.-> C

    style B fill:#e1f5fe
    style C fill:#f3e5f5
```

## Detailed Control Cycle

```mermaid
graph TD
    subgraph "Perception Layer"
        P1[Camera Processing]
        P2[Lidar Processing]
        P3[IMU Processing]
    end

    subgraph "Estimation Layer"
        E1[State Estimator]
        E2[Localization]
        E3[Mapping]
    end

    subgraph "Planning Layer"
        Pl1[Path Planner]
        Pl2[Behavior Planner]
        Pl3[Motion Controller]
    end

    subgraph "Control Layer"
        C1[High-level Controller]
        C2[Low-level Controller]
        C3[Joint Controllers]
    end

    P1 --> E1
    P2 --> E1
    P3 --> E1
    E1 --> Pl1
    E2 --> Pl2
    E3 --> Pl1
    Pl1 --> Pl2
    Pl2 --> Pl3
    Pl3 --> C1
    C1 --> C2
    C2 --> C3

    style Pl1 fill:#e8f5e8
    style C3 fill:#e3f2fd
```

## QoS Profile Recommendations

For different communication patterns in humanoid robotics:

- **Joint States**: BEST_EFFORT, KEEP_LAST(5) - Some data loss acceptable for real-time control
- **Sensor Data**: RELIABLE, KEEP_LAST(10) - Critical for safety and control
- **Control Commands**: RELIABLE, KEEP_LAST(1) - Must be delivered for safety
- **Emergency Commands**: RELIABLE, KEEP_LAST(1) - Critical for safety systems
- **Logging**: RELIABLE, KEEP_ALL - For debugging and analysis

## Key Communication Principles

1. **Decoupling**: Publishers and subscribers don't need to know about each other
2. **Real-time**: QoS profiles ensure timing requirements are met
3. **Scalability**: New nodes can subscribe to topics without changing existing code
4. **Safety**: Critical commands use reliable delivery with appropriate timeouts
5. **Modularity**: Each node has a clear responsibility in the communication flow