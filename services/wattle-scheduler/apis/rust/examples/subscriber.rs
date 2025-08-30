use wattle_rs::Wattle;
use std::env;
use tokio::time::{sleep, Duration};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("=== Wattle API Subscriber Example ===\n");

    // Set up environment variables
    unsafe {
        env::set_var("WATTLE_TASK_NAME", "data-subscriber");
        env::set_var("WATTLE_GROUP_NAME", "example-workflow");
    }

    println!("Environment setup:");
    println!("  Task Name: {}", env::var("WATTLE_TASK_NAME").unwrap_or_else(|_| "NOT_SET".to_string()));
    println!("  Group Name: {}", env::var("WATTLE_GROUP_NAME").unwrap_or_else(|_| "NOT_SET".to_string()));
    println!();

    // Try to create Wattle instance
    println!("Creating Wattle instance...");
    let wattle = match Wattle::new().await {
        Ok(w) => {
            println!("✅ Wattle instance created successfully!");
            w
        }
        Err(e) => {
            println!("❌ Failed to create Wattle instance: {}", e);
            println!("This is expected if Zenoh is not running.");
            println!("To run this example properly, start a Zenoh router:");
            println!("  zenohd");
            return Ok(());
        }
    };

    println!("\n=== Subscribing to Topics ===\n");

    // Subscribe to status updates
    println!("1. Subscribing to status updates...");
    match wattle.subscribe_json("status", |msg| {
        println!("📋 Status Update: {}", msg);
        if let Some(message) = msg.get("message") {
            println!("   Message: {}", message);
        }
        if let Some(timestamp) = msg.get("timestamp") {
            println!("   Timestamp: {}", timestamp);
        }
    }).await {
        Ok(_) => println!("✅ Subscribed to status topic"),
        Err(e) => println!("❌ Failed to subscribe to status: {}", e),
    }

    // Subscribe to progress updates  
    println!("\n2. Subscribing to progress updates...");
    match wattle.subscribe_json("progress", |msg| {
        println!("📊 Progress Update: {}", msg);
        if let Some(progress) = msg.get("progress") {
            println!("   Progress: {}%", progress);
        }
        if let Some(message) = msg.get("message") {
            println!("   Message: {}", message);
        }
    }).await {
        Ok(_) => println!("✅ Subscribed to progress topic"),
        Err(e) => println!("❌ Failed to subscribe to progress: {}", e),
    }

    println!("\n=== Listening for Messages ===");
    println!("Subscriber is now listening for messages...");
    println!("Run the publisher example in another terminal to see messages.");
    println!("Press Ctrl+C to stop.\n");

    // Keep the subscriber running
    loop {
        sleep(Duration::from_secs(1)).await;
    }
}
