# Wattle Rust SDK

Multi-language SDK for Worker-to-Worker communication using Zenoh + Apache Arrow.

## 🎯 Features

- **Unified Communication**: Pub/Sub and Request/Reply patterns using Zenoh
- **Data Formats**: Support for JSON and Apache Arrow data
- **Async/Sync APIs**: Full async support with optional sync wrappers
- **Multi-language**: Rust SDK with planned Python and C++ bindings
- **Streaming**: Support for streaming data transfer and processing
- **Key Naming**: Standardized key format: `wattle/services/<workflow>/<worker>/<service>`

## 🚀 Quick Start

### Environment Setup

```bash
export WATTLE_WORKFLOW_NAME="my_workflow"
export WATTLE_WORKER_NAME="worker_1"
```

### Basic Usage

```rust
use wattle_rs::{WattleClient, create_sample_batch};
use serde_json::json;

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> eyre::Result<()> {
    // Create client
    let client = WattleClient::new().await?;
    
    // Publish JSON data
    let data = json!({"message": "Hello, Worker!"});
    client.publish_json("my_service", &data).await?;
    
    // Publish Arrow data
    let batch = create_sample_batch()?;
    client.publish_arrow("analytics", &batch).await?;
    
    // Send request to another worker
    let response = client.request_json_simple(
        "target_worker", 
        "compute", 
        &json!({"operation": "sum", "values": [1, 2, 3]})
    ).await?;
    
    client.close().await?;
    Ok(())
}
```

## 🔧 API Reference

### Core Types

```rust
pub enum MessageType {
    Publish,    // One-way message
    Request,    // Request expecting reply
    Reply,      // Reply to request
    StreamStart,// Start of stream
    StreamChunk,// Stream data chunk
    StreamEnd,  // End of stream
}

pub enum DataFormat {
    Json,   // JSON serialized data
    Arrow,  // Apache Arrow data
    Raw,    // Raw bytes
}

pub struct WattleMessage {
    pub id: String,
    pub message_type: MessageType,
    pub format: DataFormat,
    pub data: Vec<u8>,
    pub metadata: HashMap<String, String>,
}
```

### Client Methods

#### Publishing
- `publish_json(service_name, data)` - Publish JSON data
- `publish_arrow(service_name, batch)` - Publish Arrow data

#### Requests (Simplified)
- `request_json_simple(target_worker, service, data)` - Send JSON request
- `request_arrow_simple(target_worker, service, batch)` - Send Arrow request

#### Utility
- `build_key(service_name)` - Build service key
- `build_target_key(worker, service)` - Build target service key
- `close()` - Clean shutdown

## 🏗️ Architecture

### Key Format
All communications use the standardized key format:
```
wattle/services/<workflow_name>/<worker_name>/<service_name>
```

### Message Flow
```
Worker A                    Zenoh                    Worker B
   |                          |                         |
   |-- publish_json() -------->|                         |
   |                          |-- subscription -------->|
   |                          |                         |
   |-- request_json() -------->|                         |
   |                          |-- request ------------->|
   |<-- reply -----------------|<-- reply --------------|
```

### Data Serialization
- **JSON**: Direct serde_json serialization
- **Arrow**: Custom serialization via JSON (for compatibility)
- **Metadata**: Key-value pairs for additional context

## 🧪 Testing

Run the test suite:
```bash
cargo run --bin test_sdk
```

Run the demo:
```bash
cargo run --bin demo
```

## 📋 Implementation Status

### ✅ Completed (Rust SDK)
- [x] Core WattleClient structure
- [x] JSON message publishing
- [x] Arrow data publishing
- [x] Basic request/reply patterns
- [x] Message type system
- [x] Arrow serialization support
- [x] Environment-based configuration
- [x] Unified key naming
- [x] Clean shutdown
- [x] Basic tests and demos

### 🚧 In Progress
- [ ] Advanced request/reply with timeouts
- [ ] Subscription callbacks
- [ ] Stream processing
- [ ] Data slicing
- [ ] Python SDK (maturin/pyo3)
- [ ] C++ SDK (cxx bindings)

### 📋 Future Features
- [ ] Sync API wrappers
- [ ] Connection pooling
- [ ] Metrics and monitoring
- [ ] Configuration file support
- [ ] Advanced error handling
- [ ] Schema validation

## 🔗 Dependencies

- **zenoh**: Distributed messaging
- **arrow-array**: Apache Arrow data structures
- **arrow-schema**: Arrow schemas
- **serde/serde_json**: Serialization
- **tokio**: Async runtime
- **eyre**: Error handling
- **uuid**: Unique identifiers

## 📚 Examples

See `src/bin/` for complete examples:
- `demo.rs` - Full feature demonstration
- `test_sdk.rs` - Basic functionality test

## 🤝 Contributing

The SDK is designed to be extended with additional language bindings:

1. **Python SDK**: Using maturin/pyo3 to bind to Rust core
2. **C++ SDK**: Using cxx for safe C++/Rust interop  
3. **Other languages**: Via C FFI or direct Zenoh bindings

Each SDK should maintain API compatibility and support the same key features.

---

**Built with ❤️ for the Wattle Scheduler project**
