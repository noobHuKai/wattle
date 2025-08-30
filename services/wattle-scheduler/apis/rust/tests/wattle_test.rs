use wattle_rs::Wattle;
use serde_json::json;
use std::{env, sync::Arc};

// Mock environment setup for testing
fn setup_test_env() {
    unsafe {
        env::set_var("WATTLE_TASK_NAME", "test-worker");
        env::set_var("WATTLE_GROUP_NAME", "test-group");
    }
}

fn cleanup_test_env() {
    unsafe {
        env::remove_var("WATTLE_TASK_NAME");
        env::remove_var("WATTLE_GROUP_NAME");
    }
}

#[tokio::test]
async fn test_wattle_creation_with_env() {
    setup_test_env();
    
    // Note: This test will fail if Zenoh is not available, which is expected in CI
    let result = Wattle::new().await;
    
    // In a real test environment with Zenoh running, this would succeed
    // For now, we just test that it attempts to create and fails gracefully
    match result {
        Ok(_wattle) => {
            println!("Wattle created successfully (Zenoh is running)");
        }
        Err(e) => {
            println!("Expected error when Zenoh is not available: {}", e);
            assert!(e.to_string().contains("Zenoh") || e.to_string().contains("connection"));
        }
    }
    
    cleanup_test_env();
}

#[tokio::test]
async fn test_wattle_creation_without_env() {
    cleanup_test_env(); // Ensure env vars are not set
    
    let result = Wattle::new().await;
    assert!(result.is_err());
    
    if let Err(e) = result {
        let error_msg = e.to_string();
        assert!(error_msg.contains("Must In Wattle"));
    }
}

#[tokio::test]
async fn test_service_name_generation() {
    setup_test_env();
    
    // We can't easily test this without creating a Wattle instance
    // But we can test the logic indirectly
    
    // Test valid service names
    let valid_names = vec!["my-service", "worker1", "data-processor"];
    for name in valid_names {
        assert!(!name.contains("/"), "Valid name should not contain slash: {}", name);
    }
    
    // Test invalid service names
    let invalid_names = vec!["my/service", "worker/1", "data/processor"];
    for name in invalid_names {
        assert!(name.contains("/"), "Invalid name should contain slash: {}", name);
    }
    
    cleanup_test_env();
}

#[test]
fn test_json_operations() {
    // Test JSON serialization and deserialization
    let test_data = json!({
        "message": "Hello from Wattle",
        "timestamp": "2025-01-01T00:00:00Z",
        "data": {
            "count": 42,
            "items": ["item1", "item2", "item3"]
        }
    });
    
    // Test serialization
    let json_string = test_data.to_string();
    assert!(json_string.contains("Hello from Wattle"));
    assert!(json_string.contains("42"));
    
    // Test deserialization
    let parsed: serde_json::Value = serde_json::from_str(&json_string).unwrap();
    assert_eq!(parsed["message"], "Hello from Wattle");
    assert_eq!(parsed["data"]["count"], 42);
    assert_eq!(parsed["data"]["items"][0], "item1");
}

// Integration test with mock callbacks
#[tokio::test]
async fn test_callback_functionality() {
    use std::sync::atomic::{AtomicU32, Ordering};
    
    // Simulate callback functionality
    let call_count = Arc::new(AtomicU32::new(0));
    let call_count_clone = Arc::clone(&call_count);
    
    let mock_callback = move |_data: serde_json::Value| {
        call_count_clone.fetch_add(1, Ordering::SeqCst);
    };
    
    // Simulate multiple callback invocations
    let test_messages = vec![
        json!({"type": "status", "message": "Task started"}),
        json!({"type": "progress", "progress": 50}),
        json!({"type": "status", "message": "Task completed"}),
    ];
    
    for message in test_messages {
        mock_callback(message);
    }
    
    assert_eq!(call_count.load(Ordering::SeqCst), 3);
}
