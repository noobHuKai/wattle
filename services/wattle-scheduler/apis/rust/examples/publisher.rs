use wattle_rs::Wattle;
use serde_json::json;
use std::env;
use tokio::time::{sleep, Duration};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("=== Wattle API Publisher Example ===\n");

    // Set up environment variables (normally these would be set by the Wattle scheduler)
    unsafe {
        env::set_var("WATTLE_TASK_NAME", "data-publisher");
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

    println!("\n=== Publishing Data ===\n");

    // Example 1: Simple status update
    println!("1. Publishing simple status update...");
    let status_data = json!({
        "type": "status",
        "message": "Task started successfully",
        "timestamp": std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs(),
        "worker": "data-publisher"
    });

    match wattle.publish_json("status", status_data).await {
        Ok(_) => println!("✅ Status published successfully"),
        Err(e) => println!("❌ Failed to publish status: {}", e),
    }

    sleep(Duration::from_millis(500)).await;

    // Example 2: Progress updates
    println!("\n2. Publishing progress updates...");
    for progress in [25, 50, 75, 100] {
        let progress_data = json!({
            "type": "progress",
            "progress": progress,
            "message": format!("Processing... {}% complete", progress),
            "timestamp": std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs()
        });

        match wattle.publish_json("progress", progress_data).await {
            Ok(_) => println!("  ✅ Progress {}% published", progress),
            Err(e) => println!("  ❌ Failed to publish progress {}: {}", progress, e),
        }

        sleep(Duration::from_millis(300)).await;
    }

    // Example 3: Final completion
    println!("\n3. Publishing completion status...");
    let completion_data = json!({
        "type": "completion",
        "message": "Task completed successfully",
        "timestamp": std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs()
    });

    match wattle.publish_json("status", completion_data).await {
        Ok(_) => println!("✅ Completion status published successfully"),
        Err(e) => println!("❌ Failed to publish completion status: {}", e),
    }

    println!("\n=== Publisher Example Completed ===");

    // Clean up environment
    unsafe {
        env::remove_var("WATTLE_TASK_NAME");
        env::remove_var("WATTLE_GROUP_NAME");
    }

    Ok(())
}
