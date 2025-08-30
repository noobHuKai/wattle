# Wattle Scheduler API 文档

## API 概述

Wattle Scheduler 提供完整的 RESTful API，用于工作流和工作者的管理。所有 API 都基于 JSON 格式进行数据交换，并遵循标准的 HTTP 状态码。

### 基础信息
- **Base URL**: `http://localhost:9240`
- **API Version**: `v1`
- **Content-Type**: `application/json`
- **API Prefix**: `/api`

### 统一响应格式
```json
{
  "success": true,
  "data": <响应数据>,
  "message": "操作成功",
  "timestamp": "2024-08-30T12:34:56.789Z"
}
```

### 错误响应格式
```json
{
  "success": false,
  "error": {
    "code": "WORKFLOW_NOT_FOUND",
    "message": "工作流 'example-workflow' 不存在"
  },
  "timestamp": "2024-08-30T12:34:56.789Z"
}
```

## 工作流管理 API

### 1. 列出所有工作流
```http
GET /api/workflows
```

#### 响应示例
```json
{
  "success": true,
  "data": [
    {
      "name": "data-pipeline",
      "working_dir": "/opt/workflows/data-pipeline",
      "status": "completed",
      "created_at": "2024-08-30T10:00:00.000Z",
      "started_at": "2024-08-30T10:30:00.000Z",
      "completed_at": "2024-08-30T11:00:00.000Z",
      "deleted_at": null
    },
    {
      "name": "ml-training",
      "working_dir": "/opt/workflows/ml-training",
      "status": "running",
      "created_at": "2024-08-30T11:30:00.000Z",
      "started_at": "2024-08-30T11:35:00.000Z",
      "completed_at": null,
      "deleted_at": null
    }
  ]
}
```

### 2. 获取工作流详情
```http
GET /api/workflows/{workflow_name}
```

#### 路径参数
- `workflow_name` (string): 工作流名称

#### 响应示例
```json
{
  "success": true,
  "data": {
    "name": "data-pipeline",
    "working_dir": "/opt/workflows/data-pipeline",
    "status": "completed",
    "created_at": "2024-08-30T10:00:00.000Z",
    "started_at": "2024-08-30T10:30:00.000Z",
    "completed_at": "2024-08-30T11:00:00.000Z",
    "deleted_at": null
  }
}
```

### 3. 创建工作流
```http
POST /api/workflows
```

#### 请求体
```json
{
  "name": "example-workflow",
  "working_dir": "/tmp/example-workflow",
  "workers": [
    {
      "name": "data_fetcher",
      "workflow_name": "example-workflow",
      "command": "python fetch_data.py",
      "args": ["--source", "database"],
      "working_dir": "/tmp/example-workflow",
      "env_vars": {
        "DATABASE_URL": "sqlite:///data.db"
      },
      "inputs": null,
      "outputs": {
        "raw_data": "/tmp/raw_data.json",
        "metadata": "/tmp/metadata.json"
      }
    },
    {
      "name": "data_processor",
      "workflow_name": "example-workflow",
      "command": "python process_data.py",
      "args": ["--input", "/tmp/raw_data.json"],
      "working_dir": "/tmp/example-workflow",
      "env_vars": {
        "PROCESSING_MODE": "batch"
      },
      "inputs": {
        "input_data": "data_fetcher/raw_data",
        "config": "data_fetcher/metadata"
      },
      "outputs": {
        "processed_data": "/tmp/processed_data.json"
      }
    }
  ]
}
```

#### 响应示例
```json
{
  "success": true,
  "data": "Workflow 'example-workflow' created successfully"
}
```

### 4. 启动工作流
```http
POST /api/workflows/{workflow_name}/start
```

#### 路径参数
- `workflow_name` (string): 工作流名称

#### 响应示例
```json
{
  "success": true,
  "data": "Workflow 'example-workflow' started successfully"
}
```

### 5. 获取工作流中的工作者
```http
GET /api/workflows/{workflow_name}/workers
```

#### 路径参数
- `workflow_name` (string): 工作流名称

#### 响应示例
```json
{
  "success": true,
  "data": [
    {
      "workflow_name": "example-workflow",
      "name": "data_fetcher",
      "command": "python fetch_data.py",
      "args": "[\"--source\", \"database\"]",
      "working_dir": "/tmp/example-workflow",
      "env_vars": "{\"DATABASE_URL\": \"sqlite:///data.db\"}",
      "inputs": null,
      "outputs": "{\"raw_data\": \"/tmp/raw_data.json\", \"metadata\": \"/tmp/metadata.json\"}",
      "status": "completed",
      "error_message": null,
      "created_at": "2024-08-30T10:00:00.000Z",
      "started_at": "2024-08-30T10:30:00.000Z",
      "completed_at": "2024-08-30T10:35:00.000Z",
      "deleted_at": null
    }
  ]
}
```

## 工作者管理 API

### 1. 列出工作流中的工作者
```http
GET /api/workers/{workflow_name}
```

#### 路径参数
- `workflow_name` (string): 工作流名称

#### 响应示例
```json
{
  "success": true,
  "data": [
    {
      "name": "data_fetcher",
      "workflow_name": "example-workflow",
      "status": "completed",
      "command": "python fetch_data.py",
      "created_at": "2024-08-30T10:00:00.000Z",
      "started_at": "2024-08-30T10:30:00.000Z",
      "completed_at": "2024-08-30T10:35:00.000Z"
    }
  ]
}
```

### 2. 获取工作者详情
```http
GET /api/workers/{workflow_name}/{worker_name}
```

#### 路径参数
- `workflow_name` (string): 工作流名称
- `worker_name` (string): 工作者名称

#### 响应示例
```json
{
  "success": true,
  "data": {
    "name": "data_fetcher",
    "workflow_name": "example-workflow",
    "command": "python fetch_data.py",
    "args": ["--source", "database"],
    "working_dir": "/tmp/example-workflow",
    "env_vars": {
      "DATABASE_URL": "sqlite:///data.db"
    },
    "status": "running"
  }
}
```

### 3. 获取工作者 Schema
```http
GET /api/workers/{workflow_name}/{worker_name}/schema
```

#### 响应示例
```json
{
  "success": true,
  "data": {
    "schema": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "workflow_name": {"type": "string"},
        "command": {"type": "string"},
        "args": {
          "type": "array",
          "items": {"type": "string"}
        },
        "working_dir": {"type": "string"},
        "env_vars": {
          "type": "object",
          "additionalProperties": {"type": "string"}
        }
      }
    }
  }
}
```

### 4. 启动工作者
```http
POST /api/workers/{workflow_name}/{worker_name}/start
```

#### 路径参数
- `workflow_name` (string): 工作流名称
- `worker_name` (string): 工作者名称

#### 响应示例
```json
{
  "success": true,
  "data": "Worker 'example-workflow/data_fetcher' started"
}
```

### 5. 停止工作者
```http
POST /api/workers/{workflow_name}/{worker_name}/stop
```

#### 路径参数
- `workflow_name` (string): 工作流名称
- `worker_name` (string): 工作者名称

#### 响应示例
```json
{
  "success": true,
  "data": "Worker 'example-workflow/data_fetcher' stopped"
}
```

### 6. 重启工作者
```http
POST /api/workers/{workflow_name}/{worker_name}/restart
```

#### 路径参数
- `workflow_name` (string): 工作流名称
- `worker_name` (string): 工作者名称

#### 响应示例
```json
{
  "success": true,
  "data": "Worker 'example-workflow/data_fetcher' restarted"
}
```

### 7. 获取工作者日志
```http
GET /api/workers/{workflow_name}/{worker_name}/logs
```

#### 路径参数
- `workflow_name` (string): 工作流名称
- `worker_name` (string): 工作者名称

#### 响应示例
```json
{
  "success": true,
  "data": [
    "stdout: /tmp/example-workflow/data_fetcher/stdout.log",
    "stderr: /tmp/example-workflow/data_fetcher/stderr.log"
  ]
}
```

### 8. 工作者日志流 (SSE)
```http
GET /api/workers/{workflow_name}/{worker_name}/logs/stream
```

#### 响应类型
`text/event-stream`

#### 事件格式
```
data: {"type": "stdout", "content": "Starting data fetch process..."}

data: {"type": "stderr", "content": "Warning: Connection timeout, retrying..."}

data: {"type": "stdout", "content": "Data fetch completed successfully"}
```

### 9. 获取工作者指标
```http
GET /api/workers/{workflow_name}/{worker_name}/metrics
```

#### 响应示例
```json
{
  "success": true,
  "data": {
    "metrics": {
      "cpu_usage": "15.5%",
      "memory_usage": "128MB",
      "runtime": "00:05:23",
      "status": "running"
    }
  }
}
```

### 10. 删除工作者
```http
DELETE /api/workers/{workflow_name}/{worker_name}
```

#### 路径参数
- `workflow_name` (string): 工作流名称
- `worker_name` (string): 工作者名称

#### 响应示例
```json
{
  "success": true,
  "data": "Worker 'example-workflow/data_fetcher' deleted successfully"
}
```

## 状态码说明

### 成功状态码
- `200 OK` - 请求成功
- `201 Created` - 资源创建成功
- `204 No Content` - 请求成功，无返回内容

### 客户端错误状态码
- `400 Bad Request` - 请求参数错误
- `404 Not Found` - 资源不存在
- `409 Conflict` - 资源冲突（如重复创建）
- `422 Unprocessable Entity` - 请求格式正确但语义错误

### 服务器错误状态码
- `500 Internal Server Error` - 服务器内部错误
- `503 Service Unavailable` - 服务不可用

## 数据类型定义

### Workflow
```typescript
interface Workflow {
  name: string;
  working_dir?: string;
  workers: Worker[];
}
```

### Worker
```typescript
interface Worker {
  name: string;
  workflow_name: string;
  command: string;
  args?: string[];
  working_dir?: string;
  env_vars?: Record<string, string>;
  inputs?: Record<string, string>;
  outputs?: Record<string, string>;
}
```

### WorkflowEntity
```typescript
interface WorkflowEntity {
  name: string;
  working_dir?: string;
  status: 'created' | 'running' | 'completed' | 'failed' | 'cancelled';
  created_at: string;
  deleted_at?: string;
  started_at?: string;
  completed_at?: string;
}
```

### WorkerEntity
```typescript
interface WorkerEntity {
  workflow_name: string;
  name: string;
  command: string;
  args?: string;  // JSON string
  working_dir?: string;
  env_vars?: string;  // JSON string
  inputs?: string;    // JSON string
  outputs?: string;   // JSON string
  status: 'created' | 'running' | 'completed' | 'failed' | 'cancelled';
  error_message?: string;
  created_at: string;
  deleted_at?: string;
  started_at?: string;
  completed_at?: string;
}
```

### WorkerStatus
```typescript
type WorkerStatus = 
  | 'created'
  | 'running'  
  | 'completed'
  | 'failed'
  | 'cancelled';
```

## 使用示例

### 创建并执行一个简单工作流

#### 1. 创建工作流
```bash
curl -X POST http://localhost:9240/api/workflows \
  -H "Content-Type: application/json" \
  -d '{
    "name": "hello-world",
    "working_dir": "/tmp/hello-world",
    "workers": [
      {
        "name": "greet",
        "workflow_name": "hello-world",
        "command": "echo Hello, Wattle Scheduler!",
        "args": [],
        "working_dir": "/tmp/hello-world",
        "env_vars": {},
        "inputs": null,
        "outputs": {
          "greeting": "/tmp/hello-world/greeting.txt"
        }
      }
    ]
  }'
```

#### 2. 启动工作流
```bash
curl -X POST http://localhost:9240/api/workflows/hello-world/start
```

#### 3. 查看工作流状态
```bash
curl http://localhost:9240/api/workflows/hello-world
```

#### 4. 查看工作者日志
```bash
curl http://localhost:9240/api/workers/hello-world/greet/logs
```

### 创建 DAG 工作流

```bash
curl -X POST http://localhost:9240/api/workflows \
  -H "Content-Type: application/json" \
  -d '{
    "name": "data-pipeline",
    "working_dir": "/tmp/data-pipeline",
    "workers": [
      {
        "name": "fetch_data",
        "workflow_name": "data-pipeline",
        "command": "python3 -c \"import json; print(json.dumps({\\\"data\\\": [1,2,3,4,5]}))\"",
        "working_dir": "/tmp/data-pipeline",
        "outputs": {
          "raw_data": "/tmp/data-pipeline/raw.json"
        }
      },
      {
        "name": "process_data",
        "workflow_name": "data-pipeline",
        "command": "python3 -c \"import json; data=json.load(open(\\\"/tmp/data-pipeline/raw.json\\\")); result=sum(data[\\\"data\\\"]); print(f\\\"Sum: {result}\\\")\"",
        "working_dir": "/tmp/data-pipeline",
        "inputs": {
          "input_file": "fetch_data/raw_data"
        },
        "outputs": {
          "result": "/tmp/data-pipeline/result.txt"
        }
      }
    ]
  }'
```

## 错误处理

### 常见错误场景

#### 1. 工作流不存在
```json
{
  "success": false,
  "error": {
    "code": "WORKFLOW_NOT_FOUND",
    "message": "工作流 'non-existent' 不存在"
  },
  "timestamp": "2024-08-30T12:34:56.789Z"
}
```

#### 2. 工作流已存在
```json
{
  "success": false,
  "error": {
    "code": "WORKFLOW_ALREADY_EXISTS",
    "message": "工作流 'example-workflow' 已存在"
  },
  "timestamp": "2024-08-30T12:34:56.789Z"
}
```

#### 3. 依赖关系验证失败
```json
{
  "success": false,
  "error": {
    "code": "DEPENDENCY_VALIDATION_FAILED",
    "message": "依赖验证失败: worker 'processor' 依赖 'fetcher/output' 但该输出不存在"
  },
  "timestamp": "2024-08-30T12:34:56.789Z"
}
```

#### 4. 循环依赖检测
```json
{
  "success": false,
  "error": {
    "code": "CIRCULAR_DEPENDENCY",
    "message": "检测到循环依赖，无法执行工作流"
  },
  "timestamp": "2024-08-30T12:34:56.789Z"
}
```

## 最佳实践

### 1. 工作流设计
- 使用描述性的工作流和工作者名称
- 合理设置工作目录和环境变量
- 明确定义输入输出依赖关系

### 2. 错误处理
- 始终检查 API 响应的 `success` 字段
- 使用适当的 HTTP 客户端超时设置
- 实现重试机制处理临时网络问题

### 3. 监控和日志
- 定期检查工作流和工作者状态
- 使用 SSE 获取实时日志流
- 实现告警机制监控失败任务

### 4. 性能优化
- 避免创建过深的依赖链
- 合理利用并行执行能力
- 监控资源使用情况
