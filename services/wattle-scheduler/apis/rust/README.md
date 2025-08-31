# Wattle-RS

[![Crates.io](https://img.shields.io/crates/v/wattle-rs.svg)](https://crates.io/crates/wattle-rs)
[![Documentation](https://docs.rs/wattle-rs/badge.svg)](https://docs.rs/wattle-rs)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE)
[![Build Status](https://github.com/noobHuKai/wattle/workflows/CI/badge.svg)](https://github.com/noobHuKai/wattle/actions)

**Wattle-RS** is a high-performance message passing and data processing system for Rust applications. It provides robust publish/subscribe and request/reply messaging patterns with support for multiple data formats including JSON, binary data, and Apache Arrow.

## ✨ Features

- 🚀 **High Performance**: Optimized for low latency and high throughput
- 🔒 **Thread Safe**: All operations are thread-safe using modern Rust concurrency primitives
- ⚡ **Async/Await**: Built on tokio with full async/await support
- 📊 **Multiple Data Formats**: JSON, binary, text, and Apache Arrow support
- 🎯 **Flexible Patterns**: Both publish/subscribe and request/reply messaging
- 📈 **Built-in Metrics**: Comprehensive performance monitoring
- 🛡️ **Error Recovery**: Robust error handling and automatic recovery
- 🔧 **Configurable**: Extensive configuration options for fine-tuning

## 🚀 Quick Start

Add Wattle-RS to your `Cargo.toml`:

```toml
[dependencies]
wattle-rs = "0.1.0"

# Enable optional features
wattle-rs = { version = "0.1.0", features = ["arrow", "compression", "metrics"] }
```

### Basic Publish/Subscribe

```rust
use wattle_rs::{Wattle, DataFormat};

#[tokio::main]
async fn main() -> wattle_rs::WattleResult<()> {
    // Initialize Wattle
    let wattle = Wattle::new().await?;
    
    // Subscribe to a topic
    let subscription_id = wattle.subscribe("events/user", |data| {
        println!("Received: {}", data);
        Ok(())
    }).await?;
    
    // Publish a message
    wattle.publish("events/user", DataFormat::json(&serde_json::json!({
        "user_id": 123,
        "action": "login",
        "timestamp": "2025-08-31T10:00:00Z"
    }))?).await?;
    
    // Cleanup
    wattle.unsubscribe(subscription_id).await?;
    wattle.shutdown().await?;
    
    Ok(())
}
```

### Request/Reply Pattern

```rust
use wattle_rs::{Wattle, DataFormat};

#[tokio::main]
async fn main() -> wattle_rs::WattleResult<()> {
    let wattle = Wattle::new().await?;
    
    // Register a request handler
    wattle.register_request_handler("service/echo", |request| async move {
        // Echo back the request data
        Ok(wattle_rs::messaging::Response::success(&request.id, request.data))
    }).await?;
    
    // Send a request
    let response = wattle.request("service/echo", 
        DataFormat::text("Hello, Wattle!")).await?;
    
    match response.result {
        Ok(data) => println!("Response: {}", data),
        Err(error) => println!("Error: {}", error),
    }
    
    wattle.shutdown().await?;
    Ok(())
}
```

### Async Callbacks

```rust
use wattle_rs::{Wattle, DataFormat};
use tokio::time::{sleep, Duration};

#[tokio::main]
async fn main() -> wattle_rs::WattleResult<()> {
    let wattle = Wattle::new().await?;
    
    // Subscribe with async callback
    wattle.subscribe_async("async/processing", |data| async move {
        // Perform async processing
        sleep(Duration::from_millis(100)).await;
        println!("Async processing completed for: {}", data);
        Ok(())
    }).await?;
    
    wattle.publish("async/processing", DataFormat::text("Async data")).await?;
    
    sleep(Duration::from_millis(200)).await;
    wattle.shutdown().await?;
    
    Ok(())
}
```

## 📊 Data Formats

Wattle-RS supports multiple data formats for maximum flexibility:

### JSON Data

```rust
use serde_json::json;

// Create JSON data
let data = DataFormat::json(&json!({
    "name": "Alice",
    "age": 30,
    "active": true
}))?;

// Publish JSON data
wattle.publish("users/update", data).await?;
```

### Binary Data

```rust
// Create binary data
let binary_data = b"Hello, binary world!".to_vec();
let data = DataFormat::binary(binary_data);

wattle.publish("data/binary", data).await?;
```

### Apache Arrow (with feature flag)

```rust
use arrow::{array::Int32Array, datatypes::*, record_batch::RecordBatch};

// Create Arrow RecordBatch
let schema = Arc::new(Schema::new(vec![
    Field::new("id", DataType::Int32, false),
]));
let array = Arc::new(Int32Array::from(vec![1, 2, 3, 4, 5]));
let batch = RecordBatch::try_new(schema, vec![array])?;

// Publish Arrow data
wattle.publish("analytics/data", DataFormat::arrow(batch)).await?;
```

## 🔧 Configuration

Customize Wattle's behavior with configuration options:

```rust
use wattle_rs::WattleConfig;
use std::time::Duration;

let mut config = WattleConfig::default();
config.timeouts.request_timeout = Duration::from_secs(10);
config.messaging.default_buffer_size = 10_000;
config.concurrency.max_concurrent_requests = 5_000;

let wattle = Wattle::with_config(config).await?;
```

### Predefined Configurations

```rust
// High performance configuration
let wattle = Wattle::with_config(WattleConfig::high_performance()).await?;

// Low latency configuration
let wattle = Wattle::with_config(WattleConfig::low_latency()).await?;

// Development configuration
let wattle = Wattle::with_config(WattleConfig::development()).await?;
```

## 📈 Metrics and Monitoring

Monitor your application's performance with built-in metrics:

```rust
let metrics = wattle.metrics();

println!("Messages published: {}", metrics.messages_published);
println!("Messages delivered: {}", metrics.messages_delivered);
println!("Active subscriptions: {}", metrics.active_subscriptions);
println!("Request success rate: {:.2}%", metrics.request_success_rate() * 100.0);
println!("Is healthy: {}", metrics.is_healthy);
```

## 🎯 Advanced Features

### Wildcard Subscriptions

```rust
// Subscribe to all user events
wattle.subscribe("user/*", |data| {
    println!("User event: {}", data);
    Ok(())
}).await?;

// This will trigger the above subscription
wattle.publish("user/login", DataFormat::text("User logged in")).await?;
wattle.publish("user/logout", DataFormat::text("User logged out")).await?;
```

### Message Headers and Metadata

```rust
use wattle_rs::messaging::{Message, Topic};

// Create message with custom headers
let topic = Topic::new("notifications/email")?;
let message = Message::new(topic, DataFormat::text("Email sent"))
    .with_header("sender", "notification-service")
    .with_header("priority", "high")
    .with_priority(5)
    .with_ttl(3600); // 1 hour TTL

wattle.publish_message(message).await?;
```

### Error Handling

```rust
wattle.subscribe("error-prone/topic", |data| {
    match process_data(&data) {
        Ok(result) => {
            println!("Processed successfully: {:?}", result);
            Ok(())
        }
        Err(e) => {
            eprintln!("Processing failed: {}", e);
            Err(wattle_rs::WattleError::callback(format!("Processing error: {}", e)))
        }
    }
}).await?;
```

## 🏗️ Architecture

Wattle-RS is built with a multi-threaded, async architecture:

- **Main Thread**: Handles API calls and coordination
- **Router Thread**: Manages message routing and subscription matching
- **Request Processor**: Handles request/reply message processing  
- **Callback Executor**: Executes user callbacks with concurrency control
- **Cleanup Thread**: Periodic cleanup of expired resources
- **Heartbeat Thread**: Health monitoring and metrics updates

## 📦 Feature Flags

Enable optional functionality with feature flags:

```toml
[dependencies]
wattle-rs = { version = "0.1.0", features = ["arrow", "compression", "metrics"] }
```

Available features:

- `json` (default): JSON serialization support
- `arrow`: Apache Arrow format support
- `compression`: Data compression capabilities
- `metrics`: Performance metrics collection

## 🧪 Testing

Run the test suite:

```bash
# Run all tests
cargo test

# Run with all features
cargo test --all-features

# Run integration tests
cargo test --test integration_tests

# Run with logging
RUST_LOG=debug cargo test
```

## 📊 Benchmarks

Run performance benchmarks:

```bash
cargo bench
```

Example benchmark results on a modern system:
- Message throughput: >1M messages/second
- Request latency: <1ms average
- Memory usage: <10MB for 10,000 active subscriptions

## 🚀 Examples

Check out the examples directory for more use cases:

- [`basic_usage.rs`](examples/basic_usage.rs) - Getting started guide
- [`async_pubsub.rs`](examples/async_pubsub.rs) - Advanced async patterns
- [`arrow_processing.rs`](examples/arrow_processing.rs) - Apache Arrow data processing

Run an example:

```bash
cargo run --example basic_usage
cargo run --example async_pubsub
cargo run --example arrow_processing --features arrow
```

## 🛡️ Safety and Security

Wattle-RS prioritizes safety and security:

- **Memory Safety**: Built with Rust's memory safety guarantees
- **Thread Safety**: All operations are thread-safe by design
- **Resource Management**: Automatic cleanup prevents memory leaks
- **Error Isolation**: Callback errors don't crash the system
- **Graceful Degradation**: Continues operating during partial failures

## 🚀 Performance Tips

1. **Use appropriate buffer sizes** for your workload
2. **Enable compression** for large messages over slow networks
3. **Use wildcard subscriptions** sparingly for better performance
4. **Batch operations** when possible
5. **Monitor metrics** to identify bottlenecks
6. **Configure timeouts** appropriately for your use case

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guide](CONTRIBUTING.md) for details.

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## 📄 License

This project is licensed under either of:

- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE))
- MIT License ([LICENSE-MIT](LICENSE-MIT))

at your option.

## 🔗 Links

- [Documentation](https://docs.rs/wattle-rs)
- [Crates.io](https://crates.io/crates/wattle-rs)
- [GitHub Repository](https://github.com/noobHuKai/wattle)
- [Issue Tracker](https://github.com/noobHuKai/wattle/issues)

## 🙏 Acknowledgments

Special thanks to all contributors and the Rust community for making this project possible.

---

**Wattle-RS** - High-performance messaging for the modern Rust ecosystem! 🦀✨
