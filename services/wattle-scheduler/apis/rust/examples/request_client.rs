//! Wattle Rust SDK 请求-回复示例 (客户端)

use eyre::Result;
use serde_json::json;
use tokio::time::{sleep, Duration};
use wattle_rs::{WattleClient, create_sample_batch};

#[tokio::main]
async fn main() -> Result<()> {
    // 设置环境变量
    std::env::set_var("WATTLE_WORKFLOW_NAME", "example_workflow");
    std::env::set_var("WATTLE_WORKER_NAME", "client");

    // 等待服务器启动
    sleep(Duration::from_secs(2)).await;

    // 创建客户端
    let client = WattleClient::new().await?;
    println!("Client created successfully");

    // 发送 JSON 请求
    let request_data = json!({
        "numbers": [1.5, 2.7, 3.2, 4.8, 5.1]
    });

    match client.request_json("server", "calculate", &request_data).await {
        Ok(response) => {
            println!("JSON Response: {}", serde_json::to_string_pretty(&response)?);
        },
        Err(e) => {
            eprintln!("JSON Request failed: {}", e);
        }
    }

    // 发送 Arrow 请求
    let input_batch = create_sample_batch()?;
    match client.request_arrow("server", "process", &input_batch).await {
        Ok(response_batch) => {
            println!("Arrow Response: {} rows, {} columns", 
                     response_batch.num_rows(), response_batch.num_columns());
        },
        Err(e) => {
            eprintln!("Arrow Request failed: {}", e);
        }
    }

    client.close().await?;
    println!("Client finished");
    
    Ok(())
}
