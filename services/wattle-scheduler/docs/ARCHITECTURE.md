# Wattle Scheduler 技术架构图

## 系统整体架构

```mermaid
graph TB
    subgraph "用户界面层"
        UI1[Web Frontend<br/>React + TypeScript]
        UI2[CLI Tool<br/>Command Line]
        UI3[REST API<br/>HTTP Client]
    end

    subgraph "API 网关层"
        API[API Server<br/>Axum Web Framework]
        API --> |HTTP/REST| UI1
        API --> |HTTP/REST| UI2
        API --> |HTTP/REST| UI3
    end

    subgraph "业务逻辑层"
        COORD[Coordinator<br/>工作流协调器]
        API --> COORD
    end

    subgraph "执行引擎层"
        RUNTIME[Runtime Executor<br/>任务执行引擎]
        COORD --> RUNTIME
    end

    subgraph "通信层"
        ZENOH[Zenoh P2P Network<br/>分布式通信]
        COORD -.-> ZENOH
        RUNTIME -.-> ZENOH
    end

    subgraph "存储层"
        DB[(SQLite Database<br/>数据持久化)]
        COORD --> DB
        RUNTIME --> DB
    end

    subgraph "文件系统"
        FS[Log Files<br/>日志存储]
        RUNTIME --> FS
    end
```

## 核心模块架构

```mermaid
graph TD
    subgraph "Workspace Structure"
        subgraph "crates/ (核心库)"
            CORE[core<br/>核心类型定义]
            COORD[coordinator<br/>协调器]
            RUNTIME[runtime<br/>执行引擎]
            STORAGE[storage<br/>存储层]
        end
        
        subgraph "binaries/ (应用程序)"
            API[apiserver<br/>API服务器]
            CLI[cli<br/>命令行工具]
        end
        
        subgraph "apis/ (SDK)"
            RUST_API[rust<br/>Rust SDK]
        end
    end

    CORE --> COORD
    CORE --> RUNTIME
    CORE --> STORAGE
    
    COORD --> RUNTIME
    COORD --> STORAGE
    
    API --> COORD
    CLI --> COORD
    RUST_API --> COORD
```

## DAG 执行流程

```mermaid
flowchart TD
    START([开始执行工作流])
    VALIDATE{验证依赖关系}
    TOPO[拓扑排序]
    LEVEL[获取执行层级]
    EXECUTE[并发执行当前层级]
    CHECK{是否有更多层级}
    SUCCESS([执行成功])
    FAIL([执行失败])

    START --> VALIDATE
    VALIDATE -->|验证通过| TOPO
    VALIDATE -->|验证失败| FAIL
    TOPO --> LEVEL
    LEVEL --> EXECUTE
    EXECUTE --> CHECK
    CHECK -->|有| LEVEL
    CHECK -->|无| SUCCESS
    EXECUTE -.->|任务失败| FAIL

    subgraph "并发执行详情"
        EXECUTE --> TASK1[Task 1]
        EXECUTE --> TASK2[Task 2]
        EXECUTE --> TASK3[Task N...]
        TASK1 --> WAIT[等待所有任务完成]
        TASK2 --> WAIT
        TASK3 --> WAIT
        WAIT --> CHECK
    end
```

## 数据流架构

```mermaid
sequenceDiagram
    participant Client
    participant API
    participant Coordinator
    participant Runtime
    participant Storage
    participant Zenoh

    Client->>API: 创建工作流
    API->>Coordinator: 验证并创建
    Coordinator->>Storage: 持久化工作流
    Storage-->>Coordinator: 确认保存

    Client->>API: 启动工作流
    API->>Coordinator: 执行工作流
    Coordinator->>Coordinator: DAG拓扑排序
    Coordinator->>Runtime: 执行任务层级
    Runtime->>Storage: 更新任务状态
    Runtime->>Zenoh: 发布状态变更
    Zenoh-->>Client: 实时状态推送
```

## 存储架构

```mermaid
erDiagram
    WORKFLOWS ||--o{ WORKERS : contains
    WORKERS ||--o{ WORKER_LOGS : generates

    WORKFLOWS {
        string name PK
        string working_dir
        string status
        datetime created_at
        datetime deleted_at
        datetime started_at
        datetime completed_at
    }

    WORKERS {
        string workflow_name FK
        string name PK
        string command
        json args
        string working_dir
        json env_vars
        json inputs
        json outputs
        string status
        string error_message
        datetime created_at
        datetime deleted_at
        datetime started_at
        datetime completed_at
    }

    WORKER_LOGS {
        int id PK
        string workflow_name FK
        string worker_name FK
        string log_type
        string file_path
        datetime created_at
    }
```

## 网络通信架构

```mermaid
graph TB
    subgraph "Zenoh Network Topology"
        subgraph "Node A"
            APP_A[Application A]
            ZENOH_A[Zenoh Router A]
            APP_A --> ZENOH_A
        end
        
        subgraph "Node B"
            APP_B[Application B]
            ZENOH_B[Zenoh Router B]
            APP_B --> ZENOH_B
        end
        
        subgraph "Node C"
            APP_C[Application C]
            ZENOH_C[Zenoh Router C]
            APP_C --> ZENOH_C
        end
        
        ZENOH_A <--> ZENOH_B
        ZENOH_B <--> ZENOH_C
        ZENOH_A <--> ZENOH_C
    end

    subgraph "Communication Patterns"
        PUB[Publisher<br/>发布状态]
        SUB[Subscriber<br/>订阅状态]
        QUERY[Queryable<br/>响应查询]
        GET[Get<br/>发起查询]
    end

    APP_A --> PUB
    APP_B --> SUB
    APP_C --> QUERY
    APP_A --> GET
```

## 部署架构

```mermaid
graph TB
    subgraph "Production Environment"
        subgraph "Load Balancer"
            LB[Nginx/HAProxy]
        end
        
        subgraph "Application Layer"
            API1[API Server 1]
            API2[API Server 2]
            API3[API Server N]
        end
        
        subgraph "Service Layer"
            COORD1[Coordinator 1]
            COORD2[Coordinator 2]
            COORD3[Coordinator N]
        end
        
        subgraph "Storage Layer"
            DB[(Database Cluster)]
            FS[(Shared File System)]
        end
        
        subgraph "Monitoring"
            PROM[Prometheus]
            GRAF[Grafana]
            LOG[Log Aggregation]
        end
    end

    LB --> API1
    LB --> API2
    LB --> API3
    
    API1 --> COORD1
    API2 --> COORD2
    API3 --> COORD3
    
    COORD1 --> DB
    COORD2 --> DB
    COORD3 --> DB
    
    COORD1 --> FS
    COORD2 --> FS
    COORD3 --> FS
    
    COORD1 --> PROM
    COORD2 --> PROM
    COORD3 --> PROM
    
    PROM --> GRAF
```

## 安全架构

```mermaid
graph TB
    subgraph "Security Layers"
        subgraph "Network Security"
            TLS[TLS/SSL Encryption]
            FW[Firewall Rules]
            VPN[VPN Access]
        end
        
        subgraph "Authentication & Authorization"
            AUTH[Authentication Service]
            RBAC[Role-Based Access Control]
            JWT[JWT Tokens]
        end
        
        subgraph "Data Security"
            ENCRYPT[Data Encryption]
            BACKUP[Secure Backup]
            AUDIT[Audit Logging]
        end
        
        subgraph "Runtime Security"
            SANDBOX[Process Sandboxing]
            RESOURCE[Resource Limits]
            MONITOR[Security Monitoring]
        end
    end

    TLS --> AUTH
    AUTH --> RBAC
    RBAC --> JWT
    ENCRYPT --> BACKUP
    BACKUP --> AUDIT
    SANDBOX --> RESOURCE
    RESOURCE --> MONITOR
```

## 监控架构

```mermaid
graph TB
    subgraph "Application Metrics"
        APP_METRICS[Application Metrics]
        CUSTOM_METRICS[Custom Business Metrics]
        TRACES[Distributed Traces]
    end
    
    subgraph "System Metrics"
        CPU[CPU Usage]
        MEM[Memory Usage]
        DISK[Disk I/O]
        NET[Network I/O]
    end
    
    subgraph "Collection Layer"
        PROMETHEUS[Prometheus]
        JAEGER[Jaeger Tracing]
        FLUENTD[Log Collection]
    end
    
    subgraph "Visualization Layer"
        GRAFANA[Grafana Dashboards]
        KIBANA[Kibana Logs]
        ALERTS[Alert Manager]
    end
    
    APP_METRICS --> PROMETHEUS
    CUSTOM_METRICS --> PROMETHEUS
    TRACES --> JAEGER
    
    CPU --> PROMETHEUS
    MEM --> PROMETHEUS
    DISK --> PROMETHEUS
    NET --> PROMETHEUS
    
    PROMETHEUS --> GRAFANA
    JAEGER --> GRAFANA
    FLUENTD --> KIBANA
    
    PROMETHEUS --> ALERTS
```

## 开发工作流

```mermaid
gitgraph
    commit id: "Initial"
    branch feature/dag-support
    checkout feature/dag-support
    commit id: "Add DAG types"
    commit id: "Implement validation"
    commit id: "Add topo sort"
    checkout main
    merge feature/dag-support
    commit id: "Release v0.1.0"
    
    branch feature/api-server
    checkout feature/api-server
    commit id: "Add Axum server"
    commit id: "Implement REST API"
    commit id: "Add middleware"
    checkout main
    merge feature/api-server
    commit id: "Release v0.2.0"
```
