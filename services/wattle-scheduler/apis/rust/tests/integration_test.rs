//! Wattle Rust SDK 集成测试

use eyre::Result;
use serde_json::{json, Value};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use tokio::time::{sleep, Duration, timeout};
use wattle_rs::{WattleClient, create_sample_batch};

#[tokio::test]
async fn test_publish_subscribe_json() -> Result<()> {
    // 设置环境变量
    std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
    std::env::set_var("WATTLE_WORKER_NAME", "test_worker");

    // 创建发布者和订阅者
    let publisher = WattleClient::new().await?;
    let subscriber = WattleClient::new().await?;

    // 设置消息计数器
    let received_count = Arc::new(AtomicUsize::new(0));
    let count_clone = received_count.clone();

    // 订阅消息
    subscriber.subscribe_json("test_service", move |data: Value| {
        println!("Received: {}", data);
        count_clone.fetch_add(1, Ordering::SeqCst);
    }).await?;

    // 等待订阅建立
    sleep(Duration::from_millis(100)).await;

    // 发布消息
    let test_data = json!({
        "message": "test message",
        "number": 42
    });
    
    publisher.publish_json("test_service", &test_data).await?;
    
    // 等待消息接收
    sleep(Duration::from_millis(500)).await;
    
    // 验证接收到消息
    assert_eq!(received_count.load(Ordering::SeqCst), 1);

    publisher.close().await?;
    subscriber.close().await?;

    Ok(())
}

#[tokio::test]
async fn test_publish_subscribe_arrow() -> Result<()> {
    std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
    std::env::set_var("WATTLE_WORKER_NAME", "test_worker_arrow");

    let publisher = WattleClient::new().await?;
    let subscriber = WattleClient::new().await?;

    let received_count = Arc::new(AtomicUsize::new(0));
    let count_clone = received_count.clone();

    // 订阅 Arrow 数据
    subscriber.subscribe_arrow("arrow_test", move |batch| {
        println!("Received Arrow batch: {} rows", batch.num_rows());
        count_clone.fetch_add(1, Ordering::SeqCst);
    }).await?;

    sleep(Duration::from_millis(100)).await;

    // 发布 Arrow 数据
    let batch = create_sample_batch()?;
    publisher.publish_arrow("arrow_test", &batch).await?;
    
    sleep(Duration::from_millis(500)).await;
    
    assert_eq!(received_count.load(Ordering::SeqCst), 1);

    publisher.close().await?;
    subscriber.close().await?;

    Ok(())
}

#[tokio::test]
async fn test_request_reply_json() -> Result<()> {
    std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
    
    // 创建服务器和客户端
    std::env::set_var("WATTLE_WORKER_NAME", "test_server");
    let server = WattleClient::new().await?;
    
    std::env::set_var("WATTLE_WORKER_NAME", "test_client");
    let client = WattleClient::new().await?;

    // 注册请求处理器
    server.handle_requests_json("echo", |data: Value| {
        json!({
            "echo": data,
            "server": "test_server"
        })
    }).await?;

    // 等待服务器准备就绪
    sleep(Duration::from_millis(100)).await;

    // 发送请求
    let request = json!({"message": "hello"});
    let response = client.request_json("test_server", "echo", &request).await?;

    // 验证回复
    assert_eq!(response["echo"]["message"], "hello");
    assert_eq!(response["server"], "test_server");

    server.close().await?;
    client.close().await?;

    Ok(())
}

#[tokio::test]
async fn test_request_reply_arrow() -> Result<()> {
    std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
    
    std::env::set_var("WATTLE_WORKER_NAME", "arrow_server");
    let server = WattleClient::new().await?;
    
    std::env::set_var("WATTLE_WORKER_NAME", "arrow_client");
    let client = WattleClient::new().await?;

    // 注册 Arrow 请求处理器
    server.handle_requests_arrow("process", |input_batch| {
        // 简单处理：返回相同的批次
        input_batch
    }).await?;

    sleep(Duration::from_millis(100)).await;

    // 发送 Arrow 请求
    let input_batch = create_sample_batch()?;
    let response_batch = client.request_arrow("arrow_server", "process", &input_batch).await?;

    // 验证回复
    assert_eq!(input_batch.num_rows(), response_batch.num_rows());
    assert_eq!(input_batch.num_columns(), response_batch.num_columns());

    server.close().await?;
    client.close().await?;

    Ok(())
}

#[tokio::test]
async fn test_stream_data() -> Result<()> {
    std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
    std::env::set_var("WATTLE_WORKER_NAME", "stream_test");

    let publisher = WattleClient::new().await?;
    let subscriber = WattleClient::new().await?;

    let received_count = Arc::new(AtomicUsize::new(0));
    let count_clone = received_count.clone();

    // 订阅流式数据
    subscriber.subscribe_json("stream_test", move |data: Value| {
        if data.get("chunk").is_some() {
            count_clone.fetch_add(1, Ordering::SeqCst);
        }
    }).await?;

    sleep(Duration::from_millis(100)).await;

    // 发布流式数据
    let stream_data = vec![
        json!({"chunk": 1, "data": "chunk1"}),
        json!({"chunk": 2, "data": "chunk2"}),
        json!({"chunk": 3, "data": "chunk3"}),
    ];
    
    publisher.publish_stream_json("stream_test", stream_data).await?;
    
    // 等待所有消息接收
    sleep(Duration::from_millis(1000)).await;
    
    // 验证接收到所有块
    assert_eq!(received_count.load(Ordering::SeqCst), 3);

    publisher.close().await?;
    subscriber.close().await?;

    Ok(())
}

#[tokio::test]
async fn test_key_building() {
    std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
    std::env::set_var("WATTLE_WORKER_NAME", "test_worker");

    let client = WattleClient::new().await.unwrap();
    let key = client.build_key("test_service");
    assert_eq!(key, "wattle/services/test_workflow/test_worker/test_service");

    let target_key = client.build_target_key("other_worker", "test_service");
    assert_eq!(target_key, "wattle/services/test_workflow/other_worker/test_service");
}
