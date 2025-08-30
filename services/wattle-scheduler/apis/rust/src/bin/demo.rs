//! Wattle Rust SDK 使用示例
//!
//! 展示如何使用 Wattle SDK 进行 Worker 间通信

use wattle_rs::{WattleClient, create_sample_batch};
use serde_json::json;
use eyre::Result;

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<()> {
    println!("🚀 Wattle Rust SDK Demo");
    println!("========================");
    
    // 设置环境变量
    std::env::set_var("WATTLE_WORKFLOW_NAME", "demo_workflow");
    std::env::set_var("WATTLE_WORKER_NAME", "worker_a");
    
    // 创建客户端
    let client = WattleClient::new().await?;
    println!("✅ Created Wattle client for workflow: {}", client.workflow_name);
    println!("   Worker name: {}", client.worker_name);
    
    // 1. 发布 JSON 数据
    println!("\n📤 Publishing JSON data...");
    let json_data = json!({
        "task": "data_processing",
        "input": {"file": "dataset.csv", "rows": 1000},
        "priority": "high",
        "timestamp": chrono::Utc::now().timestamp()
    });
    
    client.publish_json("data_processing", &json_data).await?;
    println!("   ✅ JSON data published to 'data_processing' service");
    
    // 2. 发布 Arrow 数据
    println!("\n📊 Publishing Arrow data...");
    let arrow_batch = create_sample_batch()?;
    println!("   Created Arrow batch with {} rows and {} columns", 
             arrow_batch.num_rows(), arrow_batch.num_columns());
    
    client.publish_arrow("analytics", &arrow_batch).await?;
    println!("   ✅ Arrow data published to 'analytics' service");
    
    // 3. 演示 client-to-client 请求（简化版）
    println!("\n🔄 Sending request to another worker...");
    let request_data = json!({
        "operation": "transform",
        "parameters": {"format": "parquet", "compression": "gzip"}
    });
    
    let response = client.request_json_simple("worker_b", "transform_service", &request_data).await?;
    println!("   ✅ Request sent, response: {}", response);
    
    // 4. 演示 Arrow 请求
    println!("\n📈 Sending Arrow request...");
    let request_batch = create_sample_batch()?;
    let response_batch = client.request_arrow_simple("worker_c", "compute_service", &request_batch).await?;
    println!("   ✅ Arrow request sent, received batch with {} rows", 
             response_batch.num_rows());
    
    // 清理
    client.close().await?;
    println!("\n✅ Client closed successfully");
    println!("\n🎉 Demo completed! The Wattle Rust SDK is working correctly.");
    
    println!("\n📋 Key Features Demonstrated:");
    println!("   • JSON message publishing");
    println!("   • Arrow data publishing");
    println!("   • Worker-to-worker requests (JSON)");
    println!("   • Worker-to-worker requests (Arrow)");
    println!("   • Unified key naming: wattle/services/<workflow>/<worker>/<service>");
    
    Ok(())
}
