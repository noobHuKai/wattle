//! Async publish/subscribe example for wattle-rs
//! 
//! This example demonstrates advanced async patterns including
//! concurrent publishing, batch operations, and error handling.

use wattle_rs::{Wattle, DataFormat, WattleConfig, WattleResult};
use tokio::time::{sleep, Duration, Instant};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

#[tokio::main]
async fn main() -> WattleResult<()> {
    // Initialize tracing with debug level for this example
    std::env::set_var("RUST_LOG", "debug");
    wattle_rs::init_tracing();
    
    println!("🚀 Starting advanced async pub/sub example");
    
    // Create Wattle with custom configuration optimized for high throughput
    let config = WattleConfig::high_performance();
    let wattle = Wattle::with_config(config).await?;
    
    // Example 1: High-throughput publishing and subscribing
    println!("\n⚡ Example 1: High-throughput messaging");
    
    let message_counter = Arc::new(AtomicUsize::new(0));
    let counter_clone = message_counter.clone();
    
    // Subscribe with async callback
    let sub_id = wattle.subscribe_async("high-throughput/data", move |data| {
        let counter = counter_clone.clone();
        async move {
            let count = counter.fetch_add(1, Ordering::Relaxed);
            
            if count % 1000 == 0 {
                println!("📈 Processed {} messages", count);
            }
            
            // Simulate some async processing
            if count % 100 == 0 {
                tokio::task::yield_now().await;
            }
            
            Ok(())
        }
    }).await?;
    
    println!("✅ Subscribed to high-throughput topic");
    
    // Publish messages concurrently
    let start_time = Instant::now();
    let publish_tasks: Vec<_> = (0..10).map(|worker_id| {
        let wattle = wattle.clone();
        tokio::spawn(async move {
            for i in 0..1000 {
                let message_data = DataFormat::json(&serde_json::json!({
                    "worker_id": worker_id,
                    "message_id": i,
                    "timestamp": chrono::Utc::now().to_rfc3339(),
                    "payload": format!("Message {} from worker {}", i, worker_id)
                }))?;
                
                wattle.publish("high-throughput/data", message_data).await?;
                
                // Small delay to prevent overwhelming
                if i % 100 == 0 {
                    tokio::task::yield_now().await;
                }
            }
            
            Ok::<(), wattle_rs::WattleError>(())
        })
    }).collect();
    
    // Wait for all publishing tasks to complete
    for task in publish_tasks {
        task.await.unwrap()?;
    }
    
    let publish_duration = start_time.elapsed();
    println!("📤 Published 10,000 messages in {:?}", publish_duration);
    
    // Wait for message processing
    sleep(Duration::from_secs(2)).await;
    
    let processed_count = message_counter.load(Ordering::Relaxed);
    println!("📥 Processed {} messages", processed_count);
    
    // Example 2: Error handling and recovery
    println!("\n🔧 Example 2: Error handling and recovery");
    
    let error_counter = Arc::new(AtomicUsize::new(0));
    let success_counter = Arc::new(AtomicUsize::new(0));
    
    let error_counter_clone = error_counter.clone();
    let success_counter_clone = success_counter.clone();
    
    wattle.subscribe_async("error/test", move |data| {
        let error_counter = error_counter_clone.clone();
        let success_counter = success_counter_clone.clone();
        
        async move {
            match data {
                DataFormat::Text(text) if text.contains("error") => {
                    error_counter.fetch_add(1, Ordering::Relaxed);
                    Err(wattle_rs::WattleError::callback("Simulated error"))
                }
                _ => {
                    success_counter.fetch_add(1, Ordering::Relaxed);
                    Ok(())
                }
            }
        }
    }).await?;
    
    // Publish mix of successful and error messages
    for i in 0..100 {
        let message = if i % 10 == 0 {
            DataFormat::text(format!("error message {}", i))
        } else {
            DataFormat::text(format!("success message {}", i))
        };
        
        wattle.publish("error/test", message).await?;
    }
    
    sleep(Duration::from_millis(500)).await;
    
    println!("✅ Successful messages: {}", success_counter.load(Ordering::Relaxed));
    println!("❌ Failed messages: {}", error_counter.load(Ordering::Relaxed));
    
    // Example 3: Multiple topic patterns
    println!("\n🎯 Example 3: Multiple topic patterns");
    
    let pattern_counter = Arc::new(AtomicUsize::new(0));
    let counter_clone = pattern_counter.clone();
    
    // Subscribe to all user events
    wattle.subscribe_async("user/*", move |data| {
        let counter = counter_clone.clone();
        async move {
            let count = counter.fetch_add(1, Ordering::Relaxed);
            println!("👤 User event #{}: {}", count + 1, data);
            Ok(())
        }
    }).await?;
    
    let system_counter = Arc::new(AtomicUsize::new(0));
    let system_counter_clone = system_counter.clone();
    
    // Subscribe to all system events
    wattle.subscribe_async("system/*", move |data| {
        let counter = system_counter_clone.clone();
        async move {
            let count = counter.fetch_add(1, Ordering::Relaxed);
            println!("🖥️  System event #{}: {}", count + 1, data);
            Ok(())
        }
    }).await?;
    
    // Publish to various topics
    let topics = [
        "user/login", "user/logout", "user/profile_update",
        "system/startup", "system/shutdown", "system/health_check",
        "other/event", // This won't match any pattern
    ];
    
    for topic in &topics {
        wattle.publish(*topic, DataFormat::text(format!("Event from {}", topic))).await?;
    }
    
    sleep(Duration::from_millis(200)).await;
    
    // Example 4: Request/Response with concurrent handlers
    println!("\n🔄 Example 4: Concurrent request handling");
    
    // Register multiple handlers for different services
    wattle.register_request_handler("service/math", |request| async move {
        sleep(Duration::from_millis(10)).await; // Simulate processing time
        
        match request.data {
            DataFormat::Json(json) => {
                if let (Some(a), Some(b)) = (json.get("a"), json.get("b")) {
                    if let (Some(a), Some(b)) = (a.as_f64(), b.as_f64()) {
                        let result = DataFormat::json(&serde_json::json!({
                            "operation": "add",
                            "result": a + b
                        }))?;
                        return Ok(wattle_rs::messaging::Response::success(&request.id, result));
                    }
                }
            }
            _ => {}
        }
        
        Ok(wattle_rs::messaging::Response::error(&request.id, "Invalid math request"))
    }).await?;
    
    wattle.register_request_handler("service/string", |request| async move {
        sleep(Duration::from_millis(5)).await; // Simulate processing time
        
        match request.data {
            DataFormat::Text(text) => {
                let result = DataFormat::text(text.to_uppercase());
                Ok(wattle_rs::messaging::Response::success(&request.id, result))
            }
            _ => Ok(wattle_rs::messaging::Response::error(&request.id, "Expected text data")),
        }
    }).await?;
    
    // Send concurrent requests
    let request_tasks: Vec<_> = (0..20).map(|i| {
        let wattle = wattle.clone();
        tokio::spawn(async move {
            if i % 2 == 0 {
                // Math service request
                let request_data = DataFormat::json(&serde_json::json!({
                    "a": i as f64,
                    "b": (i + 1) as f64
                }))?;
                wattle.request("service/math", request_data).await
            } else {
                // String service request
                let request_data = DataFormat::text(format!("hello world {}", i));
                wattle.request("service/string", request_data).await
            }
        })
    }).collect();
    
    let mut successful_requests = 0;
    for task in request_tasks {
        match task.await.unwrap() {
            Ok(response) => {
                if response.is_success() {
                    successful_requests += 1;
                }
            }
            Err(e) => println!("❌ Request failed: {}", e),
        }
    }
    
    println!("✅ Successful requests: {}/20", successful_requests);
    
    // Example 5: Resource monitoring
    println!("\n📊 Example 5: Resource monitoring");
    
    let metrics = wattle.metrics();
    println!("📈 Performance metrics:");
    println!("   Messages published: {}", metrics.messages_published);
    println!("   Messages delivered: {}", metrics.messages_delivered);
    println!("   Active subscriptions: {}", metrics.active_subscriptions);
    println!("   Requests sent: {}", metrics.requests_sent);
    println!("   Responses received: {}", metrics.responses_received);
    println!("   Callbacks executed: {}", metrics.callbacks_executed);
    println!("   Message success rate: {:.2}%", metrics.message_success_rate() * 100.0);
    println!("   Request success rate: {:.2}%", metrics.request_success_rate() * 100.0);
    println!("   Callback success rate: {:.2}%", metrics.callback_success_rate() * 100.0);
    
    // Example 6: Graceful shutdown with cleanup
    println!("\n🧹 Example 6: Graceful shutdown");
    
    println!("📊 Final metrics before shutdown:");
    let final_metrics = wattle.metrics();
    println!("   Total messages published: {}", final_metrics.messages_published);
    println!("   Total messages delivered: {}", final_metrics.messages_delivered);
    println!("   Final active subscriptions: {}", final_metrics.active_subscriptions);
    println!("   Uptime: {} seconds", final_metrics.uptime_seconds);
    
    // Unsubscribe from topics
    wattle.unsubscribe(sub_id).await?;
    println!("✅ Unsubscribed from high-throughput topic");
    
    // Shutdown gracefully
    let shutdown_start = Instant::now();
    wattle.shutdown().await?;
    let shutdown_duration = shutdown_start.elapsed();
    
    println!("👋 Wattle shutdown completed in {:?}", shutdown_duration);
    println!("✅ Advanced async pub/sub example completed successfully!");
    
    Ok(())
}
