//! Basic usage example for wattle-rs
//! 
//! This example demonstrates the core features of wattle-rs including
//! publish/subscribe and request/response patterns.

use wattle_rs::{Wattle, DataFormat, WattleResult};
use tokio::time::{sleep, Duration};

#[tokio::main]
async fn main() -> WattleResult<()> {
    // Initialize tracing
    wattle_rs::init_tracing();
    
    println!("🚀 Starting Wattle basic usage example");
    
    // Create a new Wattle instance
    let mut wattle = Wattle::new().await?;
    
    // Example 1: Basic Publish/Subscribe
    println!("\n📡 Example 1: Basic Publish/Subscribe");
    
    // Subscribe to a topic
    let subscription_id = wattle.subscribe("events/user", |data| {
        println!("📩 Received message: {}", data);
        Ok(())
    }).await?;
    
    println!("✅ Subscribed to 'events/user' with ID: {}", subscription_id);
    
    // Publish a JSON message
    let json_data = DataFormat::json(&serde_json::json!({
        "user_id": 123,
        "action": "login",
        "timestamp": "2025-08-31T10:00:00Z"
    }))?;
    
    let route_info = wattle.publish("events/user", json_data).await?;
    println!("📤 Published message to {} subscribers", route_info.subscriber_count);
    
    // Wait for message processing
    sleep(Duration::from_millis(100)).await;
    
    // Example 2: Text and Binary Messages
    println!("\n📝 Example 2: Text and Binary Messages");
    
    // Subscribe to a different topic
    wattle.subscribe("data/stream", |data| {
        match data {
            DataFormat::Text(text) => println!("📄 Text: {}", text),
            DataFormat::Binary(bytes) => println!("🔢 Binary: {} bytes", bytes.len()),
            _ => println!("📦 Other format: {}", data),
        }
        Ok(())
    }).await?;
    
    // Publish text message
    wattle.publish("data/stream", DataFormat::text("Hello, Wattle!")).await?;
    
    // Publish binary message
    let binary_data = b"Binary data example".to_vec();
    wattle.publish("data/stream", DataFormat::binary(binary_data)).await?;
    
    sleep(Duration::from_millis(100)).await;
    
    // Example 3: Request/Response Pattern
    println!("\n🔄 Example 3: Request/Response Pattern");
    
    // Register a request handler
    wattle.register_request_handler("service/echo", |request| async move {
        println!("🔧 Processing request: {}", request.id);
        
        // Echo back the request data with some modification
        let response_data = match request.data {
            DataFormat::Text(text) => DataFormat::text(format!("Echo: {}", text)),
            DataFormat::Json(json) => {
                let mut response_json = json;
                response_json["echoed"] = serde_json::Value::Bool(true);
                DataFormat::Json(response_json)
            }
            other => other,
        };
        
        Ok(wattle_rs::messaging::Response::success(&request.id, response_data))
    }).await?;
    
    println!("✅ Registered echo service handler");
    
    // Send a request
    let request_data = DataFormat::text("Hello from client!");
    let response = wattle.request("service/echo", request_data).await?;
    
    match response.result {
        Ok(data) => println!("✅ Received response: {}", data),
        Err(error) => println!("❌ Request failed: {}", error),
    }
    
    // Example 4: Async Callbacks
    println!("\n⚡ Example 4: Async Callbacks");
    
    wattle.subscribe_async("async/processing", |data| async move {
        println!("🔄 Starting async processing...");
        
        // Simulate async work
        sleep(Duration::from_millis(50)).await;
        
        println!("✅ Async processing completed for: {}", data);
        Ok(())
    }).await?;
    
    // Publish to trigger async callback
    wattle.publish("async/processing", DataFormat::text("Async data")).await?;
    
    sleep(Duration::from_millis(200)).await;
    
    // Example 5: Metrics and Health
    println!("\n📊 Example 5: Metrics and Health");
    
    let metrics = wattle.metrics();
    println!("📈 Messages published: {}", metrics.messages_published);
    println!("📈 Messages delivered: {}", metrics.messages_delivered);
    println!("📈 Active subscriptions: {}", metrics.active_subscriptions);
    println!("📈 Request success rate: {:.2}%", metrics.request_success_rate() * 100.0);
    println!("🟢 Wattle is healthy: {}", wattle.is_healthy());
    
    // Example 6: Wildcard Subscriptions
    println!("\n🌟 Example 6: Wildcard Subscriptions");
    
    wattle.subscribe("events/*", |data| {
        println!("🎯 Wildcard subscriber received: {}", data);
        Ok(())
    }).await?;
    
    // These should all trigger the wildcard subscription
    wattle.publish("events/user", DataFormat::text("User event")).await?;
    wattle.publish("events/system", DataFormat::text("System event")).await?;
    wattle.publish("events/notification", DataFormat::text("Notification")).await?;
    
    sleep(Duration::from_millis(100)).await;
    
    // Example 7: Message Headers and Metadata
    println!("\n🏷️  Example 7: Message Headers and Metadata");
    
    wattle.subscribe("metadata/test", |data| {
        println!("📋 Received metadata message: {}", data);
        Ok(())
    }).await?;
    
    // Create a message with custom headers
    let topic = wattle_rs::messaging::Topic::new("metadata/test")?;
    let message = wattle_rs::messaging::Message::new(topic, DataFormat::text("Message with metadata"))
        .with_header("sender", "example")
        .with_header("priority", "high")
        .with_priority(5)
        .with_ttl(3600);
    
    wattle.publish_message(message).await?;
    
    sleep(Duration::from_millis(100)).await;
    
    // Cleanup
    println!("\n🧹 Cleaning up...");
    wattle.unsubscribe(subscription_id).await?;
    
    // Final metrics
    let final_metrics = wattle.metrics();
    println!("\n📊 Final Statistics:");
    println!("   Messages published: {}", final_metrics.messages_published);
    println!("   Messages delivered: {}", final_metrics.messages_delivered);
    println!("   Active subscriptions: {}", final_metrics.active_subscriptions);
    println!("   Callback success rate: {:.2}%", final_metrics.callback_success_rate() * 100.0);
    println!("   Uptime: {} seconds", final_metrics.uptime_seconds);
    
    // Graceful shutdown
    println!("\n👋 Shutting down Wattle...");
    wattle.shutdown().await?;
    
    println!("✅ Example completed successfully!");
    
    Ok(())
}
