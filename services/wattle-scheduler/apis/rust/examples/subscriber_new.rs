//! Wattle Rust SDK 订阅者示例

use eyre::Result;
use serde_json::Value;
use tokio::time::Duration;
use wattle_rs::WattleClient;

#[tokio::main]
async fn main() -> Result<()> {
    // 设置环境变量
    std::env::set_var("WATTLE_WORKFLOW_NAME", "example_workflow");
    std::env::set_var("WATTLE_WORKER_NAME", "subscriber");

    // 创建客户端
    let client = WattleClient::new().await?;
    println!("Subscriber client created successfully");

    // 订阅 JSON 数据
    client.subscribe_json("data_feed", |data: Value| {
        println!("Received JSON: {}", serde_json::to_string_pretty(&data).unwrap());
    }).await?;

    // 订阅 Arrow 数据
    client.subscribe_arrow("arrow_feed", |batch| {
        println!("Received Arrow batch: {} rows, {} columns", 
                 batch.num_rows(), batch.num_columns());
        
        // 打印列名
        for field in batch.schema().fields() {
            println!("  Column: {} ({})", field.name(), field.data_type());
        }
    }).await?;

    println!("Subscriber running... Press Ctrl+C to stop");
    
    // 等待中断信号
    tokio::signal::ctrl_c().await?;
    
    client.close().await?;
    println!("Subscriber stopped");
    
    Ok(())
}
