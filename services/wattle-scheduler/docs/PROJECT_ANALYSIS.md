# Wattle Scheduler 项目全面分析

## 项目概述

**Wattle Scheduler** 是一个基于 Rust 开发的分布式工作流调度器，支持 DAG（有向无环图）工作流的创建、管理和执行。项目采用现代化的微服务架构，具备高性能、高并发和高可扩展性的特点。

### 核心特性
- ✅ DAG 工作流支持：支持复杂的依赖关系管理和并行执行
- ✅ 分布式通信：基于 Zenoh 的发布订阅模式
- ✅ 多种执行模式：支持顺序、并行和基于依赖的执行
- ✅ 持久化存储：使用 SQLite 进行数据持久化
- ✅ RESTful API：完整的 HTTP API 接口
- ✅ 实时监控：支持实时日志流和状态监控
- ✅ CLI 工具：命令行工具用于工作流管理
- ✅ Web 前端：现代化的 React Web 界面

## 系统架构

### 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        Wattle Scheduler                        │
├─────────────────┬─────────────────┬─────────────────┬───────────┤
│   Web Frontend  │   CLI Client    │   External API  │   Zenoh   │
│   (React/TS)    │   (Rust)        │   (HTTP/REST)   │   (P2P)   │
├─────────────────┼─────────────────┼─────────────────┼───────────┤
│                 │                 │                 │           │
│                 │                 ▼                 │           │
│                 │       ┌─────────────────┐         │           │
│                 │       │   API Server    │◄────────┤           │
│                 │       │   (Axum Web)    │         │           │
│                 │       └─────────┬───────┘         │           │
│                 │                 │                 │           │
│                 │                 ▼                 │           │
│                 │       ┌─────────────────┐         │           │
│                 └──────►│   Coordinator   │◄────────┤           │
│                         │  (Core Engine)  │         │           │
│                         └─────────┬───────┘         │           │
│                                   │                 │           │
│          ┌────────────────────────┼─────────────────┘           │
│          │                        │                             │
│          ▼                        ▼                             │
│ ┌─────────────────┐     ┌─────────────────┐                    │
│ │   Runtime       │     │   Storage       │                    │
│ │   Executor      │     │   Repository    │                    │
│ │                 │     │                 │                    │
│ │ - Task Exec     │     │ - SQLite DB     │                    │
│ │ - Process Mgmt  │     │ - Migrations    │                    │
│ │ - Log Streaming │     │ - Entity Models │                    │
│ └─────────────────┘     └─────────────────┘                    │
└─────────────────────────────────────────────────────────────────┘
```

### 模块架构

#### 1. 核心模块 (crates/)

##### core/
- **功能**：定义核心数据类型和配置
- **主要组件**：
  - `Worker`: 工作任务定义
  - `Workflow`: 工作流定义
  - `WorkerStatus`: 任务状态枚举
  - DAG 依赖验证和拓扑排序算法

##### coordinator/
- **功能**：工作流协调器，系统核心组件
- **主要功能**：
  - 工作流生命周期管理
  - DAG 执行调度
  - 并发控制和依赖管理
  - Zenoh 通信管理

##### runtime/
- **功能**：任务执行引擎
- **主要功能**：
  - 进程创建和管理
  - 日志收集和流式传输
  - 资源监控
  - 异常处理

##### storage/
- **功能**：数据持久化层
- **主要功能**：
  - SQLite 数据库操作
  - 实体模型定义
  - 数据迁移管理
  - CRUD 操作接口

#### 2. 应用层 (binaries/)

##### apiserver/
- **功能**：HTTP API 服务器
- **技术栈**：Axum Web Framework
- **API 端点**：
  - `/api/workflows/*` - 工作流管理
  - `/api/workers/*` - 工作者管理
  - 支持 SSE 实时数据流

##### cli/
- **功能**：命令行工具
- **主要命令**：
  - `list` - 列出所有工作流
  - `create` - 创建新工作流
  - `run` - 执行工作流
  - `status` - 查看状态

#### 3. API 层 (apis/)

##### rust/
- **功能**：Rust SDK
- **主要功能**：
  - Zenoh 客户端封装
  - 发布订阅模式支持
  - JSON 数据序列化/反序列化

## 技术栈详析

### 后端技术栈

#### 核心框架
- **Rust 2021 Edition**: 系统编程语言，保证内存安全和高性能
- **Tokio**: 异步运行时，支持高并发处理
- **Eyre**: 错误处理框架
- **Serde**: 序列化/反序列化框架

#### Web 服务
- **Axum**: 现代化 Web 框架，支持异步处理
- **Tower**: 中间件支持
- **Hyper**: HTTP 实现

#### 数据库
- **SQLx**: 异步 SQL 工具包
- **SQLite**: 嵌入式数据库
- **数据库迁移**: 版本化 schema 管理

#### 通信协议
- **Zenoh**: 下一代分布式通信框架
  - 支持发布订阅模式
  - P2P 网络架构
  - 低延迟数据传输

#### 图算法
- **petgraph**: 图数据结构和算法库
  - DAG 构建和验证
  - 拓扑排序
  - 循环依赖检测

#### 日志和监控
- **tracing**: 结构化日志和追踪
- **tracing-subscriber**: 日志订阅者
- **color-eyre**: 错误报告美化

### 前端技术栈

#### 核心框架
- **React 19**: 现代化 UI 框架
- **TypeScript**: 类型安全的 JavaScript
- **Vite**: 快速构建工具

#### 路由和状态管理
- **TanStack Router**: 文件系统路由
- **TanStack Query**: 数据获取和缓存

#### UI 组件
- **Radix UI**: 无头组件库
- **Tailwind CSS**: 原子化 CSS 框架
- **Recharts**: 数据可视化图表库
- **Sonner**: Toast 通知组件

### 开发工具

#### 构建和包管理
- **Cargo**: Rust 包管理器
- **Workspace**: 多包项目管理
- **Make**: 构建脚本

#### 代码质量
- **rustfmt**: Rust 代码格式化
- **Clippy**: Rust 代码检查器

## 核心功能详解

### 1. DAG 工作流管理

#### 工作流定义
```rust
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Workflow {
    pub name: String,
    pub working_dir: Option<String>,
    pub workers: Vec<Worker>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Worker {
    pub name: String,
    pub workflow_name: String,
    pub command: String,
    pub args: Option<Vec<String>>,
    pub working_dir: Option<String>,
    pub env_vars: Option<HashMap<String, String>>,
    pub inputs: Option<HashMap<String, String>>,   // 输入依赖
    pub outputs: Option<HashMap<String, String>>,  // 输出定义
}
```

#### 依赖关系管理
- **输入依赖格式**: `worker_name/output_key`
- **依赖验证**: 确保所有依赖的输出存在
- **循环依赖检测**: 使用拓扑排序算法检测

#### 执行策略
1. **拓扑排序**: 基于依赖关系排序任务
2. **分层执行**: 同一层级的任务并发执行
3. **错误处理**: 任何任务失败将终止整个工作流

### 2. 分布式通信

#### Zenoh 集成
- **自动启动**: 系统自动检测和启动 zenohd 守护进程
- **端口管理**: 动态分配可用端口 (20000-30000 range)
- **跨平台支持**: Linux, macOS, Windows

#### 通信模式
- **发布订阅**: 支持主题订阅和消息发布
- **查询响应**: 支持请求-响应模式
- **数据流**: 支持实时数据流传输

### 3. 任务执行引擎

#### 进程管理
- **异步执行**: 基于 Tokio 的异步进程管理
- **资源隔离**: 独立的工作目录和环境变量
- **PID 跟踪**: 进程标识符映射管理

#### 日志系统
- **实时日志流**: stdout/stderr 实时收集
- **文件存储**: 日志文件按工作流和工作者分类存储
- **SSE 支持**: 支持服务器推送事件的实时日志流

#### 生命周期管理
```rust
pub enum WorkerStatus {
    Created,      // 已创建
    Running,      // 运行中
    Completed,    // 已完成
    Failed,       // 失败
    Cancelled,    // 已取消
}
```

### 4. 数据持久化

#### 数据模型
```sql
-- 工作流表
CREATE TABLE workflows (
    name TEXT PRIMARY KEY,
    working_dir TEXT,
    status TEXT NOT NULL DEFAULT 'created',
    created_at TEXT NOT NULL,
    deleted_at TEXT,
    started_at TEXT,
    completed_at TEXT
);

-- 工作者表
CREATE TABLE workers (
    workflow_name TEXT NOT NULL,
    name TEXT NOT NULL,
    command TEXT NOT NULL,
    args TEXT,              -- JSON
    working_dir TEXT,
    env_vars TEXT,          -- JSON
    inputs TEXT,            -- JSON
    outputs TEXT,           -- JSON
    status TEXT NOT NULL DEFAULT 'created',
    error_message TEXT,
    created_at TEXT NOT NULL,
    deleted_at TEXT,
    started_at TEXT,
    completed_at TEXT,
    PRIMARY KEY (workflow_name, name),
    FOREIGN KEY (workflow_name) REFERENCES workflows(name)
);
```

#### 数据迁移
- **版本化管理**: 使用 SQLx migrations
- **自动迁移**: 应用启动时自动执行迁移
- **向后兼容**: 支持数据库 schema 演进

### 5. RESTful API

#### 工作流管理 API
```http
GET    /api/workflows                    # 列出所有工作流
POST   /api/workflows                    # 创建工作流
GET    /api/workflows/{name}             # 获取工作流详情
POST   /api/workflows/{name}/start       # 启动工作流
GET    /api/workflows/{name}/workers     # 获取工作流中的工作者
```

#### 工作者管理 API
```http
GET    /api/workers/{workflow}/{worker}             # 获取工作者详情
POST   /api/workers/{workflow}/{worker}/start       # 启动工作者
POST   /api/workers/{workflow}/{worker}/stop        # 停止工作者
GET    /api/workers/{workflow}/{worker}/logs        # 获取日志
GET    /api/workers/{workflow}/{worker}/logs/stream # SSE 日志流
DELETE /api/workers/{workflow}/{worker}             # 删除工作者
```

## 项目特色与优势

### 1. 高性能设计
- **异步架构**: 全面基于 Tokio 异步运行时
- **内存安全**: Rust 语言保证内存安全
- **零拷贝**: 高效的数据传输机制
- **并发执行**: 支持同层级任务并发执行

### 2. 分布式能力
- **P2P 通信**: 基于 Zenoh 的对等网络
- **服务发现**: 自动服务发现和连接
- **负载均衡**: 分布式任务调度
- **容错机制**: 节点故障自动恢复

### 3. 开发友好
- **类型安全**: 强类型系统减少运行时错误
- **结构化日志**: 详细的追踪和调试信息
- **API 优先**: RESTful API 设计
- **文档完整**: 详细的 API 文档和示例

### 4. 可扩展性
- **模块化设计**: 清晰的模块边界
- **插件架构**: 支持功能扩展
- **多语言支持**: SDK 支持多种编程语言
- **云原生**: 容器化部署友好

### 5. 运维友好
- **实时监控**: 实时状态和指标监控
- **日志聚合**: 集中式日志管理
- **故障恢复**: 自动故障检测和恢复
- **配置管理**: 灵活的配置管理机制

## 部署和运行

### 系统要求
- **操作系统**: Linux, macOS, Windows
- **Rust**: 1.70.0+
- **Node.js**: 18+ (前端开发)

### 编译和启动
```bash
# 编译项目
cargo build --release

# 启动 API 服务器
./target/release/apiserver --config configs/config.toml

# 使用 CLI
./target/release/cli list
./target/release/cli create --input workflow.json
./target/release/cli run <workflow_name>
```

### 配置文件
```toml
[server]
host = "localhost"
port = 9240

[coordinator]
mode = "DependencyBased"  # Sequential, Parallel, DependencyBased
db_url = "configs/wattle.db"

[execution]
max_concurrent_workers = 10
log_directory = "logs/"
```

## 未来发展方向

### 短期目标
1. **监控仪表板**: 实时监控和报警系统
2. **任务模板**: 预定义的任务模板库
3. **用户权限**: 基于角色的访问控制
4. **API 网关**: 统一的 API 入口

### 长期规划
1. **多集群支持**: 跨集群工作流调度
2. **机器学习集成**: ML 工作流优化
3. **可视化编辑器**: 拖拽式工作流设计器
4. **企业级特性**: 审计、合规、安全加强

## 总结

Wattle Scheduler 是一个功能完整、架构先进的分布式工作流调度系统。它结合了 Rust 的性能优势、现代化的异步架构、分布式通信能力和友好的开发体验，为复杂的工作流管理提供了强大而灵活的解决方案。

项目的模块化设计和清晰的架构边界使其具备良好的可维护性和扩展性，是企业级工作流调度的理想选择。
