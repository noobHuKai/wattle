use wattle_rs::WattleClient;
use serde_json::json;

#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() {
    println!("Testing Wattle Rust SDK...");
    
    // 设置环境变量
    std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
    std::env::set_var("WATTLE_WORKER_NAME", "test_worker");
    
    // 测试客户端创建
    match WattleClient::new().await {
        Ok(client) => {
            println!("✅ Client created successfully");
            
            // 测试 JSON 发布
            let data = json!({"test": "data", "timestamp": 123456});
            match client.publish_json("test_service", &data).await {
                Ok(()) => println!("✅ JSON publish successful"),
                Err(e) => println!("❌ JSON publish failed: {}", e),
            }
            
            // 测试 Arrow 发布
            match wattle_rs::create_sample_batch() {
                Ok(batch) => {
                    match client.publish_arrow("arrow_service", &batch).await {
                        Ok(()) => println!("✅ Arrow publish successful"),
                        Err(e) => println!("❌ Arrow publish failed: {}", e),
                    }
                },
                Err(e) => println!("❌ Failed to create sample batch: {}", e),
            }
            
            // 关闭客户端
            match client.close().await {
                Ok(()) => println!("✅ Client closed successfully"),
                Err(e) => println!("❌ Failed to close client: {}", e),
            }
        },
        Err(e) => {
            println!("❌ Failed to create client: {}", e);
        }
    }
    
    println!("✅ All basic tests completed!");
}
