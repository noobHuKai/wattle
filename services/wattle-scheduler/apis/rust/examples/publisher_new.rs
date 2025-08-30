//! Wattle Rust SDK 发布者示例

use eyre::Result;
use serde_json::json;
use tokio::time::{sleep, Duration};
use wattle_rs::{WattleClient, create_sample_batch};

#[tokio::main]
async fn main() -> Result<()> {
    // 设置环境变量
    std::env::set_var("WATTLE_WORKFLOW_NAME", "example_workflow");
    std::env::set_var("WATTLE_WORKER_NAME", "publisher");

    // 创建客户端
    let client = WattleClient::new().await?;
    println!("Publisher client created successfully");

    // 发布 JSON 数据
    for i in 0..10 {
        let data = json!({
            "message": format!("Hello from publisher #{}", i),
            "timestamp": chrono::Utc::now().to_rfc3339(),
            "data": {
                "value": i * 10,
                "status": "active"
            }
        });

        client.publish_json("data_feed", &data).await?;
        println!("Published JSON message #{}", i);
        
        sleep(Duration::from_secs(2)).await;
    }

    // 发布 Arrow 数据
    let batch = create_sample_batch()?;
    client.publish_arrow("arrow_feed", &batch).await?;
    println!("Published Arrow batch with {} rows", batch.num_rows());

    // 发布流式数据
    let stream_data = vec![
        json!({"chunk": 1, "data": "First chunk"}),
        json!({"chunk": 2, "data": "Second chunk"}),
        json!({"chunk": 3, "data": "Third chunk"}),
    ];
    
    // 使用 publish_json 逐个发布数据块
    for chunk in stream_data {
        client.publish_json("stream_feed", &chunk).await?;
        println!("Published chunk: {}", chunk);
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    }
    println!("Published all stream data");

    // 保持运行
    println!("Publisher running... Press Ctrl+C to stop");
    tokio::signal::ctrl_c().await?;
    
    client.close().await?;
    println!("Publisher stopped");
    
    Ok(())
}
