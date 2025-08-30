# 🚀 Wattle SDK 快速启动指南

## 📦 安装和配置

### 前置要求
```bash
# 安装 Rust (如果尚未安装)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# 安装 Python (推荐 3.9+)
# 确保有 pip 和 venv

# 安装 Zenoh router (可选，用于测试)
cargo install zenohd
```

### 环境配置
```bash
# 设置必需的环境变量
export WATTLE_WORKFLOW_NAME="your_workflow"
export WATTLE_WORKER_NAME="your_worker"

# 可选配置
export ZENOH_CONFIG_URL="tcp/localhost:7447"  # 默认: tcp/localhost:7447
```

## 🦀 Rust SDK 使用

### 快速开始
```bash
cd apis/rust
cargo run --bin demo
```

### 在你的项目中使用
```toml
[dependencies]
wattle-rs = { path = "../path/to/apis/rust" }
tokio = { version = "1", features = ["full"] }
```

```rust
use wattle_rs::WattleClient;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = WattleClient::new("my_workflow", "my_worker").await?;
    
    // 发布数据
    client.publish_json("data_service", &serde_json::json!({
        "message": "Hello from Rust!"
    })).await?;
    
    // 请求数据
    let response = client.request_json(
        "other_worker", 
        "compute_service", 
        &serde_json::json!({"input": 42}),
        Some(30) // 30秒超时
    ).await?;
    
    println!("Got response: {}", response);
    Ok(())
}
```

## 🐍 Python SDK 使用

### 当前状态
- ✅ **Mock 实现**: 完整 API 设计，演示所有功能
- 🚧 **Rust 绑定**: 准备实现中 (使用 maturin + PyO3)

### 快速测试
```bash
cd apis/python
python3 examples/demo.py
```

### 在你的项目中使用 (目标 API)
```python
import asyncio
from wattle_py import WattleClient

async def main():
    async with WattleClient("my_workflow", "my_worker") as client:
        # 发布数据
        await client.publish_json("data_service", {"message": "Hello from Python!"})
        
        # 请求数据
        response = await client.request_json(
            "other_worker", 
            "compute_service", 
            {"input": 42},
            timeout=30
        )
        
        print(f"Got response: {response}")
        
        # 订阅数据
        def handle_data(data):
            print(f"Received: {data}")
            
        await client.subscribe_json("service", handle_data)
        await asyncio.sleep(10)  # 监听10秒

if __name__ == "__main__":
    asyncio.run(main())
```

## 📊 Apache Arrow 使用示例

### Rust Arrow 处理
```rust
use arrow_array::{RecordBatch, Int32Array, StringArray};
use arrow_schema::{Schema, Field, DataType};

// 创建数据
let schema = Schema::new(vec![
    Field::new("id", DataType::Int32, false),
    Field::new("name", DataType::Utf8, false),
]);

let batch = RecordBatch::try_new(
    Arc::new(schema),
    vec![
        Arc::new(Int32Array::from(vec![1, 2, 3])),
        Arc::new(StringArray::from(vec!["Alice", "Bob", "Charlie"])),
    ],
)?;

// 发送 Arrow 数据
client.publish_arrow("bulk_data", &batch).await?;
```

### Python Arrow 处理 (计划)
```python
import pyarrow as pa

# 创建数据
schema = pa.schema([
    ('id', pa.int32()),
    ('name', pa.string())
])

batch = pa.record_batch([
    pa.array([1, 2, 3]),
    pa.array(['Alice', 'Bob', 'Charlie'])
], schema=schema)

# 发送 Arrow 数据
await client.publish_arrow("bulk_data", batch)
```

## 🔧 高级用法

### 1. 订阅模式
```rust
// 持续监听多个服务
let client = WattleClient::new("workflow", "worker").await?;

// 订阅数据流
client.subscribe_json("status_updates", |data| {
    println!("Status update: {}", data);
}).await?;

// 订阅 Arrow 批处理
client.subscribe_arrow("batch_processing", |batch| {
    println!("Processing {} rows", batch.num_rows());
}).await?;

// 保持服务运行
tokio::signal::ctrl_c().await?;
```

### 2. 错误处理
```rust
use wattle_rs::{WattleClient, WattleError};

match client.request_json("worker", "service", &data, Some(10)).await {
    Ok(response) => println!("Success: {}", response),
    Err(WattleError::Timeout) => println!("Request timed out"),
    Err(WattleError::NetworkError(e)) => println!("Network error: {}", e),
    Err(e) => println!("Other error: {}", e),
}
```

### 3. 批量操作
```rust
// 并发发布多个消息
let tasks: Vec<_> = vec!["service1", "service2", "service3"]
    .into_iter()
    .map(|service| {
        let client = client.clone();
        let data = serde_json::json!({"service": service});
        tokio::spawn(async move {
            client.publish_json(service, &data).await
        })
    })
    .collect();

// 等待所有完成
futures::future::try_join_all(tasks).await?;
```

## 🧪 测试和调试

### 启动 Zenoh Router (可选)
```bash
# 在单独终端中运行
zenohd
```

### 运行测试
```bash
# Rust 测试
cd apis/rust
cargo test

# 运行所有演示
cargo run --bin demo
cargo run --bin advanced_demo
cargo run --bin test_sdk

# Python 测试
cd apis/python
python3 examples/demo.py
```

### 调试技巧
1. **检查环境变量**: `echo $WATTLE_WORKFLOW_NAME $WATTLE_WORKER_NAME`
2. **Zenoh 连接**: 确保 router 可访问 (默认 `tcp/localhost:7447`)
3. **日志输出**: 设置 `RUST_LOG=debug` 获取详细日志
4. **网络检查**: 确保防火墙允许 Zenoh 端口

## 🎯 实际部署建议

### 1. 生产环境配置
```bash
# 生产环境变量
export WATTLE_WORKFLOW_NAME="production_workflow"
export WATTLE_WORKER_NAME="worker_$(hostname)"
export ZENOH_CONFIG_URL="tcp/zenoh-router.internal:7447"
export RUST_LOG="wattle_rs=info"
```

### 2. Docker 容器化
```dockerfile
FROM rust:1.70 as builder
COPY . .
RUN cargo build --release

FROM debian:bookworm-slim
COPY --from=builder /target/release/your_app /usr/local/bin/
ENV WATTLE_WORKFLOW_NAME=production_workflow
ENV RUST_LOG=info
CMD ["your_app"]
```

### 3. Kubernetes 部署
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wattle-worker
spec:
  replicas: 3
  template:
    spec:
      containers:
      - name: worker
        image: your-registry/wattle-worker:latest
        env:
        - name: WATTLE_WORKFLOW_NAME
          value: "production_workflow"
        - name: WATTLE_WORKER_NAME
          valueFrom:
            fieldRef:
              fieldPath: metadata.name
        - name: ZENOH_CONFIG_URL
          value: "tcp/zenoh-service:7447"
```

## 📚 更多资源

- 📖 **完整文档**: [apis/rust/README.md](rust/README.md)
- 🔍 **实现细节**: [apis/rust/IMPLEMENTATION_SUMMARY.md](rust/IMPLEMENTATION_SUMMARY.md)  
- 💻 **示例代码**: [apis/rust/src/bin/](rust/src/bin/)
- 🐍 **Python 示例**: [apis/python/examples/](python/examples/)

## 🤝 贡献指南

1. **Issue 报告**: 使用 GitHub Issues
2. **代码贡献**: 遵循现有代码风格
3. **测试要求**: 新功能必须包含测试
4. **文档更新**: 重要变更需要更新文档

---

**🎊 恭喜！Wattle Scheduler 现在拥有了强大的多语言 Worker 通信能力！**
