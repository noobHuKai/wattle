//! Integration tests for wattle-rs
//! 
//! These tests verify the complete functionality of the wattle-rs library
//! including pub/sub, request/reply, and various data formats.

use wattle_rs::{Wattle, DataFormat, WattleConfig, WattleResult};
use tokio::time::{sleep, timeout, Duration};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

#[tokio::test]
async fn test_basic_pubsub() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    let received_count = Arc::new(AtomicUsize::new(0));
    let received_data = Arc::new(parking_lot::Mutex::new(Vec::new()));
    
    let count_clone = received_count.clone();
    let data_clone = received_data.clone();
    
    // Subscribe to topic
    let sub_id = wattle.subscribe("test/topic", move |data| {
        count_clone.fetch_add(1, Ordering::Relaxed);
        data_clone.lock().push(data.to_text().unwrap_or_default());
        Ok(())
    }).await?;
    
    // Publish messages
    wattle.publish("test/topic", DataFormat::text("message 1")).await?;
    wattle.publish("test/topic", DataFormat::text("message 2")).await?;
    wattle.publish("test/topic", DataFormat::text("message 3")).await?;
    
    // Wait for processing
    sleep(Duration::from_millis(100)).await;
    
    // Verify results
    assert_eq!(received_count.load(Ordering::Relaxed), 3);
    let data = received_data.lock();
    assert_eq!(data.len(), 3);
    assert!(data.contains(&"message 1".to_string()));
    assert!(data.contains(&"message 2".to_string()));
    assert!(data.contains(&"message 3".to_string()));
    
    // Cleanup
    wattle.unsubscribe(sub_id).await?;
    wattle.shutdown().await?;
    
    Ok(())
}

#[tokio::test]
async fn test_async_callbacks() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    let processed_count = Arc::new(AtomicUsize::new(0));
    let count_clone = processed_count.clone();
    
    // Subscribe with async callback
    let _sub_id = wattle.subscribe_async("async/test", move |data| {
        let count = count_clone.clone();
        async move {
            // Simulate async processing
            sleep(Duration::from_millis(10)).await;
            count.fetch_add(1, Ordering::Relaxed);
            
            // Verify data
            assert!(!data.to_text()?.is_empty());
            Ok(())
        }
    }).await?;
    
    // Publish messages
    for i in 0..10 {
        wattle.publish("async/test", DataFormat::text(format!("async message {}", i))).await?;
    }
    
    // Wait for async processing
    sleep(Duration::from_millis(200)).await;
    
    assert_eq!(processed_count.load(Ordering::Relaxed), 10);
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_request_response() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    
    // Register request handler
    wattle.register_request_handler("service/echo", |request| async move {
        // Echo back the request data
        Ok(wattle_rs::messaging::Response::success(&request.id, request.data))
    }).await?;
    
    // Send request
    let request_data = DataFormat::text("hello world");
    let response = wattle.request("service/echo", request_data.clone()).await?;
    
    // Verify response
    assert!(response.is_success());
    let response_data = response.data().unwrap();
    assert_eq!(response_data.to_text()?, "hello world");
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_request_timeout() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    
    // Register slow handler
    wattle.register_request_handler("service/slow", |request| async move {
        // Simulate slow processing
        sleep(Duration::from_secs(2)).await;
        Ok(wattle_rs::messaging::Response::success(&request.id, DataFormat::text("done")))
    }).await?;
    
    // Send request with short timeout
    let result = wattle.request_with_timeout(
        "service/slow", 
        DataFormat::text("test"), 
        Duration::from_millis(100)
    ).await;
    
    // Should timeout
    assert!(result.is_err());
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_wildcard_subscriptions() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    let received_topics = Arc::new(parking_lot::Mutex::new(Vec::new()));
    let topics_clone = received_topics.clone();
    
    // Subscribe to wildcard pattern
    let _sub_id = wattle.subscribe("events/*", move |_data| {
        topics_clone.lock().push("wildcard".to_string());
        Ok(())
    }).await?;
    
    // Subscribe to specific topic
    let topics_clone2 = received_topics.clone();
    let _sub_id2 = wattle.subscribe("events/user", move |_data| {
        topics_clone2.lock().push("specific".to_string());
        Ok(())
    }).await?;
    
    // Publish to matching topic
    wattle.publish("events/user", DataFormat::text("user event")).await?;
    wattle.publish("events/system", DataFormat::text("system event")).await?;
    wattle.publish("other/topic", DataFormat::text("other event")).await?;
    
    sleep(Duration::from_millis(100)).await;
    
    let topics = received_topics.lock();
    // events/user should trigger both wildcard and specific
    // events/system should trigger only wildcard
    // other/topic should trigger nothing
    assert!(topics.len() >= 3); // At least 3 messages received
    assert!(topics.contains(&"wildcard".to_string()));
    assert!(topics.contains(&"specific".to_string()));
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_json_data_format() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    let received_data = Arc::new(parking_lot::Mutex::new(None));
    let data_clone = received_data.clone();
    
    // Subscribe to JSON messages
    let _sub_id = wattle.subscribe("json/test", move |data| {
        match data {
            DataFormat::Json(json) => {
                *data_clone.lock() = Some(json);
                Ok(())
            }
            _ => Err(wattle_rs::WattleError::data_format("Expected JSON data")),
        }
    }).await?;
    
    // Create and publish JSON data
    let json_data = serde_json::json!({
        "user_id": 123,
        "name": "Alice",
        "active": true,
        "scores": [95, 87, 91]
    });
    
    wattle.publish("json/test", DataFormat::Json(json_data.clone())).await?;
    
    sleep(Duration::from_millis(100)).await;
    
    // Verify received data
    let received = received_data.lock();
    assert!(received.is_some());
    let received_json = received.as_ref().unwrap();
    assert_eq!(received_json["user_id"], 123);
    assert_eq!(received_json["name"], "Alice");
    assert_eq!(received_json["active"], true);
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_binary_data_format() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    let received_data = Arc::new(parking_lot::Mutex::new(None));
    let data_clone = received_data.clone();
    
    // Subscribe to binary messages
    let _sub_id = wattle.subscribe("binary/test", move |data| {
        match data {
            DataFormat::Binary(bytes) => {
                *data_clone.lock() = Some(bytes);
                Ok(())
            }
            _ => Err(wattle_rs::WattleError::data_format("Expected binary data")),
        }
    }).await?;
    
    // Create and publish binary data
    let binary_data = b"Hello, binary world!".to_vec();
    wattle.publish("binary/test", DataFormat::binary(binary_data.clone())).await?;
    
    sleep(Duration::from_millis(100)).await;
    
    // Verify received data
    let received = received_data.lock();
    assert!(received.is_some());
    assert_eq!(*received.as_ref().unwrap(), binary_data);
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_concurrent_operations() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    let processed_count = Arc::new(AtomicUsize::new(0));
    
    // Subscribe to messages
    let count_clone = processed_count.clone();
    let _sub_id = wattle.subscribe("concurrent/test", move |_data| {
        count_clone.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }).await?;
    
    // Publish messages concurrently
    let publish_tasks: Vec<_> = (0..100).map(|i| {
        let wattle = wattle.clone();
        tokio::spawn(async move {
            wattle.publish("concurrent/test", DataFormat::text(format!("message {}", i))).await
        })
    }).collect();
    
    // Wait for all tasks to complete
    for task in publish_tasks {
        task.await.unwrap()?;
    }
    
    // Wait for processing
    sleep(Duration::from_millis(200)).await;
    
    assert_eq!(processed_count.load(Ordering::Relaxed), 100);
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_error_handling() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    let error_count = Arc::new(AtomicUsize::new(0));
    let success_count = Arc::new(AtomicUsize::new(0));
    
    let error_clone = error_count.clone();
    let success_clone = success_count.clone();
    
    // Subscribe with callback that sometimes fails
    let _sub_id = wattle.subscribe("error/test", move |data| {
        match data.to_text() {
            Ok(text) if text.contains("error") => {
                error_clone.fetch_add(1, Ordering::Relaxed);
                Err(wattle_rs::WattleError::callback("Simulated callback error"))
            }
            _ => {
                success_clone.fetch_add(1, Ordering::Relaxed);
                Ok(())
            }
        }
    }).await?;
    
    // Publish mix of normal and error-triggering messages
    for i in 0..20 {
        let message = if i % 5 == 0 {
            format!("error message {}", i)
        } else {
            format!("normal message {}", i)
        };
        wattle.publish("error/test", DataFormat::text(message)).await?;
    }
    
    sleep(Duration::from_millis(200)).await;
    
    // Verify error handling
    assert_eq!(error_count.load(Ordering::Relaxed), 4); // Every 5th message
    assert_eq!(success_count.load(Ordering::Relaxed), 16); // Remaining messages
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_metrics_collection() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    
    // Subscribe and publish to generate metrics
    let _sub_id = wattle.subscribe("metrics/test", |_data| Ok(())).await?;
    
    for i in 0..10 {
        wattle.publish("metrics/test", DataFormat::text(format!("message {}", i))).await?;
    }
    
    sleep(Duration::from_millis(100)).await;
    
    // Check metrics
    let metrics = wattle.metrics();
    assert_eq!(metrics.messages_published, 10);
    assert_eq!(metrics.messages_delivered, 10);
    assert_eq!(metrics.active_subscriptions, 1);
    assert!(metrics.is_healthy);
    assert_eq!(metrics.message_success_rate(), 1.0);
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_configuration() -> WattleResult<()> {
    let mut config = WattleConfig::default();
    config.messaging.default_buffer_size = 500;
    config.timeouts.request_timeout = Duration::from_millis(100);
    
    let wattle = Wattle::with_config(config.clone()).await?;
    
    // Verify configuration
    let wattle_config = wattle.config();
    assert_eq!(wattle_config.messaging.default_buffer_size, 500);
    assert_eq!(wattle_config.timeouts.request_timeout, Duration::from_millis(100));
    
    wattle.shutdown().await?;
    Ok(())
}

#[tokio::test]
async fn test_graceful_shutdown() -> WattleResult<()> {
    let wattle = Wattle::new().await?;
    
    // Subscribe to topic
    let _sub_id = wattle.subscribe("shutdown/test", |_data| Ok(())).await?;
    
    // Verify wattle is healthy
    assert!(wattle.is_healthy());
    
    // Shutdown
    let shutdown_result = timeout(Duration::from_secs(5), wattle.shutdown()).await;
    assert!(shutdown_result.is_ok());
    assert!(shutdown_result.unwrap().is_ok());
    
    // Verify shutdown state
    assert!(!wattle.is_healthy());
    
    Ok(())
}

#[tokio::test]
#[cfg(feature = "arrow")]
async fn test_arrow_data_format() -> WattleResult<()> {
    use arrow::{
        array::{Int32Array, StringArray},
        datatypes::{DataType, Field, Schema},
        record_batch::RecordBatch,
    };
    use std::sync::Arc;
    
    let wattle = Wattle::new().await?;
    let received_batch = Arc::new(parking_lot::Mutex::new(None));
    let batch_clone = received_batch.clone();
    
    // Subscribe to Arrow messages
    let _sub_id = wattle.subscribe("arrow/test", move |data| {
        match data {
            DataFormat::Arrow(batch) => {
                *batch_clone.lock() = Some(batch);
                Ok(())
            }
            _ => Err(wattle_rs::WattleError::data_format("Expected Arrow data")),
        }
    }).await?;
    
    // Create Arrow RecordBatch
    let schema = Arc::new(Schema::new(vec![
        Field::new("id", DataType::Int32, false),
        Field::new("name", DataType::Utf8, false),
    ]));
    
    let id_array = Arc::new(Int32Array::from(vec![1, 2, 3]));
    let name_array = Arc::new(StringArray::from(vec!["Alice", "Bob", "Charlie"]));
    
    let batch = RecordBatch::try_new(schema, vec![id_array, name_array]).unwrap();
    
    // Publish Arrow data
    wattle.publish("arrow/test", DataFormat::arrow(batch.clone())).await?;
    
    sleep(Duration::from_millis(100)).await;
    
    // Verify received data
    let received = received_batch.lock();
    assert!(received.is_some());
    let received_batch = received.as_ref().unwrap();
    assert_eq!(received_batch.num_rows(), 3);
    assert_eq!(received_batch.num_columns(), 2);
    
    wattle.shutdown().await?;
    Ok(())
}
