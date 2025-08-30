//! Wattle Rust SDK 高级功能演示
//!
//! 展示真实的 Request/Reply 和订阅功能

use wattle_rs::{WattleClient, create_sample_batch};
use serde_json::json;
use eyre::Result;
use std::sync::Arc;
use tokio::time::{sleep, Duration};

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> Result<()> {
    println!("🚀 Wattle Rust SDK - 高级功能演示");
    println!("===================================");
    
    // 创建两个不同的 worker 客户端
    std::env::set_var("WATTLE_WORKFLOW_NAME", "advanced_demo");
    
    // Worker A - 请求方
    std::env::set_var("WATTLE_WORKER_NAME", "worker_a");
    let client_a = WattleClient::new().await?;
    println!("✅ Worker A 客户端创建成功");
    
    // Worker B - 服务提供方
    std::env::set_var("WATTLE_WORKER_NAME", "worker_b");
    let client_b = WattleClient::new().await?;
    println!("✅ Worker B 客户端创建成功");
    
    // 演示 1: 订阅功能
    println!("\n📡 演示 1: 数据订阅");
    let received_messages = Arc::new(tokio::sync::RwLock::new(Vec::new()));
    let received_clone = received_messages.clone();
    
    // Worker B 订阅来自 Worker A 的消息
    client_b.subscribe_json("notifications", move |data| {
        let received = received_clone.clone();
        tokio::spawn(async move {
            println!("   📥 Worker B 收到消息: {}", data);
            received.write().await.push(data);
        });
    }).await?;
    
    println!("   🔄 Worker B 开始监听 'notifications' 服务");
    
    // 等待订阅生效
    sleep(Duration::from_millis(200)).await;
    
    // Worker A 发布消息
    for i in 1..=3 {
        let notification = json!({
            "type": "alert",
            "message": format!("系统通知 #{}", i),
            "priority": if i == 3 { "high" } else { "normal" },
            "timestamp": chrono::Utc::now().timestamp()
        });
        
        client_a.publish_json("notifications", &notification).await?;
        println!("   📤 Worker A 发送通知 #{}", i);
        sleep(Duration::from_millis(100)).await;
    }
    
    // 等待消息处理
    sleep(Duration::from_millis(300)).await;
    
    let received_count = received_messages.read().await.len();
    println!("   ✅ Worker B 成功接收了 {} 条消息", received_count);
    
    // 演示 2: Request/Reply with timeout（模拟）
    println!("\n🔄 演示 2: Request/Reply 超时测试");
    
    let request_data = json!({
        "operation": "complex_calculation",
        "input": {"numbers": [1, 2, 3, 4, 5]},
        "timeout": "short"
    });
    
    // 发送请求到不存在的服务（会超时）
    println!("   📤 Worker A 向不存在的服务发送请求...");
    let start_time = std::time::Instant::now();
    
    match client_a.request_json("nonexistent_worker", "calc_service", &request_data, Some(2)).await {
        Ok(_) => println!("   ❌ 意外收到响应"),
        Err(e) => {
            let elapsed = start_time.elapsed();
            if e.to_string().contains("timeout") {
                println!("   ✅ 请求正确超时，耗时: {:.1}秒", elapsed.as_secs_f64());
            } else {
                println!("   ❌ 请求失败，原因: {}", e);
            }
        }
    }
    
    // 演示 3: Arrow 数据订阅
    println!("\n📊 演示 3: Arrow 数据订阅");
    let arrow_received = Arc::new(tokio::sync::RwLock::new(0));
    let arrow_received_clone = arrow_received.clone();
    
    // Worker B 订阅 Arrow 数据
    client_b.subscribe_arrow("data_stream", move |batch| {
        let counter = arrow_received_clone.clone();
        tokio::spawn(async move {
            let mut count = counter.write().await;
            *count += 1;
            println!("   📊 Worker B 收到 Arrow 批次 #{}, 行数: {}, 列数: {}", 
                     *count, batch.num_rows(), batch.num_columns());
        });
    }).await?;
    
    println!("   🔄 Worker B 开始监听 Arrow 数据流");
    sleep(Duration::from_millis(200)).await;
    
    // Worker A 发送 Arrow 数据
    for i in 1..=2 {
        let batch = create_sample_batch()?;
        client_a.publish_arrow("data_stream", &batch).await?;
        println!("   📈 Worker A 发送 Arrow 批次 #{}", i);
        sleep(Duration::from_millis(150)).await;
    }
    
    sleep(Duration::from_millis(300)).await;
    let arrow_count = *arrow_received.read().await;
    println!("   ✅ Worker B 成功接收了 {} 个 Arrow 批次", arrow_count);
    
    // 演示 4: 高级 Request/Reply（简化版调用真实版）
    println!("\n🔧 演示 4: 高级 Request/Reply API");
    
    let advanced_request = json!({
        "service": "data_transformation",
        "params": {
            "input_format": "csv",
            "output_format": "parquet",
            "compression": "gzip"
        },
        "metadata": {
            "client_id": "worker_a",
            "request_time": chrono::Utc::now().timestamp()
        }
    });
    
    println!("   📤 使用高级 API 发送请求（带自定义超时）...");
    
    // 使用新的高级 request_json 方法（会超时因为没有服务端）
    match client_a.request_json("worker_b", "transform_service", &advanced_request, Some(3)).await {
        Ok(response) => println!("   ✅ 收到响应: {}", response),
        Err(e) => {
            if e.to_string().contains("timeout") {
                println!("   ⏰ 请求超时（正常，因为没有服务端响应）");
            } else {
                println!("   ❌ 请求错误: {}", e);
            }
        }
    }
    
    // 清理资源
    client_a.close().await?;
    client_b.close().await?;
    
    println!("\n🎉 高级功能演示完成！");
    println!("\n📋 演示的高级功能:");
    println!("   ✅ JSON 数据实时订阅");
    println!("   ✅ Arrow 数据实时订阅");
    println!("   ✅ 超时控制的 Request/Reply");
    println!("   ✅ 多 Worker 协作通信");
    println!("   ✅ 优雅的资源清理");
    
    Ok(())
}
