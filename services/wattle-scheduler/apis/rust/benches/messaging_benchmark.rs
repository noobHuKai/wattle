//! Messaging performance benchmarks for wattle-rs

use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId, Throughput};
use wattle_rs::{Wattle, DataFormat, WattleConfig};
use std::sync::{Arc, atomic::{AtomicUsize, Ordering}};
use tokio::time::Duration;

fn pubsub_benchmark(c: &mut Criterion) {
    let rt = tokio::runtime::Runtime::new().unwrap();
    
    let mut group = c.benchmark_group("pubsub");
    
    for message_count in [100, 1000, 10000] {
        group.throughput(Throughput::Elements(message_count));
        
        group.bench_with_input(
            BenchmarkId::new("publish_text", message_count),
            &message_count,
            |b, &message_count| {
                b.to_async(&rt).iter(|| async {
                    let wattle = Wattle::new().await.unwrap();
                    let counter = Arc::new(AtomicUsize::new(0));
                    let counter_clone = counter.clone();
                    
                    // Subscribe
                    wattle.subscribe("bench/topic", move |_data| {
                        counter_clone.fetch_add(1, Ordering::Relaxed);
                        Ok(())
                    }).await.unwrap();
                    
                    // Benchmark publishing
                    let start = std::time::Instant::now();
                    for i in 0..message_count {
                        let data = DataFormat::text(format!("message {}", i));
                        wattle.publish("bench/topic", data).await.unwrap();
                    }
                    
                    // Wait for processing
                    while counter.load(Ordering::Relaxed) < message_count as usize {
                        tokio::task::yield_now().await;
                    }
                    
                    let duration = start.elapsed();
                    wattle.shutdown().await.unwrap();
                    
                    black_box(duration);
                });
            }
        );
        
        group.bench_with_input(
            BenchmarkId::new("publish_json", message_count),
            &message_count,
            |b, &message_count| {
                b.to_async(&rt).iter(|| async {
                    let wattle = Wattle::with_config(WattleConfig::high_performance()).await.unwrap();
                    let counter = Arc::new(AtomicUsize::new(0));
                    let counter_clone = counter.clone();
                    
                    // Subscribe
                    wattle.subscribe("bench/json", move |_data| {
                        counter_clone.fetch_add(1, Ordering::Relaxed);
                        Ok(())
                    }).await.unwrap();
                    
                    // Benchmark publishing JSON
                    let start = std::time::Instant::now();
                    for i in 0..message_count {
                        let data = DataFormat::json(&serde_json::json!({
                            "id": i,
                            "message": format!("JSON message {}", i),
                            "timestamp": chrono::Utc::now().to_rfc3339()
                        })).unwrap();
                        wattle.publish("bench/json", data).await.unwrap();
                    }
                    
                    // Wait for processing
                    while counter.load(Ordering::Relaxed) < message_count as usize {
                        tokio::task::yield_now().await;
                    }
                    
                    let duration = start.elapsed();
                    wattle.shutdown().await.unwrap();
                    
                    black_box(duration);
                });
            }
        );
    }
    
    group.finish();
}

fn request_response_benchmark(c: &mut Criterion) {
    let rt = tokio::runtime::Runtime::new().unwrap();
    
    let mut group = c.benchmark_group("request_response");
    
    for request_count in [10, 100, 1000] {
        group.throughput(Throughput::Elements(request_count));
        
        group.bench_with_input(
            BenchmarkId::new("echo_service", request_count),
            &request_count,
            |b, &request_count| {
                b.to_async(&rt).iter(|| async {
                    let wattle = Wattle::with_config(WattleConfig::low_latency()).await.unwrap();
                    
                    // Register echo handler
                    wattle.register_request_handler("bench/echo", |request| async move {
                        Ok(wattle_rs::messaging::Response::success(&request.id, request.data))
                    }).await.unwrap();
                    
                    // Benchmark requests
                    let start = std::time::Instant::now();
                    
                    let mut handles = Vec::new();
                    for i in 0..request_count {
                        let wattle = wattle.clone();
                        let handle = tokio::spawn(async move {
                            let data = DataFormat::text(format!("request {}", i));
                            wattle.request("bench/echo", data).await
                        });
                        handles.push(handle);
                    }
                    
                    // Wait for all requests
                    for handle in handles {
                        handle.await.unwrap().unwrap();
                    }
                    
                    let duration = start.elapsed();
                    wattle.shutdown().await.unwrap();
                    
                    black_box(duration);
                });
            }
        );
    }
    
    group.finish();
}

fn concurrent_subscribers_benchmark(c: &mut Criterion) {
    let rt = tokio::runtime::Runtime::new().unwrap();
    
    let mut group = c.benchmark_group("concurrent_subscribers");
    
    for subscriber_count in [10, 100, 1000] {
        group.throughput(Throughput::Elements(subscriber_count));
        
        group.bench_with_input(
            BenchmarkId::new("multiple_subscribers", subscriber_count),
            &subscriber_count,
            |b, &subscriber_count| {
                b.to_async(&rt).iter(|| async {
                    let wattle = Wattle::with_config(WattleConfig::high_performance()).await.unwrap();
                    let total_counter = Arc::new(AtomicUsize::new(0));
                    
                    // Create multiple subscribers
                    for i in 0..subscriber_count {
                        let counter = total_counter.clone();
                        wattle.subscribe(&format!("bench/multi/{}", i), move |_data| {
                            counter.fetch_add(1, Ordering::Relaxed);
                            Ok(())
                        }).await.unwrap();
                    }
                    
                    let start = std::time::Instant::now();
                    
                    // Publish to all topics
                    for i in 0..subscriber_count {
                        let data = DataFormat::text(format!("broadcast {}", i));
                        wattle.publish(&format!("bench/multi/{}", i), data).await.unwrap();
                    }
                    
                    // Wait for all processing
                    while total_counter.load(Ordering::Relaxed) < subscriber_count as usize {
                        tokio::task::yield_now().await;
                    }
                    
                    let duration = start.elapsed();
                    wattle.shutdown().await.unwrap();
                    
                    black_box(duration);
                });
            }
        );
    }
    
    group.finish();
}

fn data_format_benchmark(c: &mut Criterion) {
    let rt = tokio::runtime::Runtime::new().unwrap();
    
    let mut group = c.benchmark_group("data_format");
    
    // Benchmark different data format conversions
    group.bench_function("json_creation", |b| {
        b.iter(|| {
            let data = serde_json::json!({
                "id": 123,
                "name": "benchmark",
                "values": [1, 2, 3, 4, 5],
                "nested": {
                    "key": "value",
                    "number": 42
                }
            });
            black_box(DataFormat::Json(data));
        });
    });
    
    group.bench_function("binary_creation", |b| {
        b.iter(|| {
            let data = b"This is a test binary message for benchmarking".to_vec();
            black_box(DataFormat::binary(data));
        });
    });
    
    group.bench_function("text_creation", |b| {
        b.iter(|| {
            let data = "This is a test text message for benchmarking purposes".to_string();
            black_box(DataFormat::text(data));
        });
    });
    
    // Benchmark format conversions
    let json_data = DataFormat::json(&serde_json::json!({
        "message": "test conversion performance"
    })).unwrap();
    
    group.bench_function("json_to_text", |b| {
        b.iter(|| {
            black_box(json_data.to_text().unwrap());
        });
    });
    
    group.bench_function("json_to_binary", |b| {
        b.iter(|| {
            black_box(json_data.to_binary().unwrap());
        });
    });
    
    let text_data = DataFormat::text("Test message for conversion benchmarking");
    
    group.bench_function("text_to_json", |b| {
        b.iter(|| {
            black_box(text_data.to_json().unwrap());
        });
    });
    
    group.finish();
}

fn memory_usage_benchmark(c: &mut Criterion) {
    let rt = tokio::runtime::Runtime::new().unwrap();
    
    let mut group = c.benchmark_group("memory_usage");
    
    group.bench_function("wattle_creation", |b| {
        b.to_async(&rt).iter(|| async {
            let wattle = Wattle::new().await.unwrap();
            wattle.shutdown().await.unwrap();
            black_box(wattle);
        });
    });
    
    group.bench_function("subscription_lifecycle", |b| {
        b.to_async(&rt).iter(|| async {
            let wattle = Wattle::new().await.unwrap();
            
            let sub_id = wattle.subscribe("bench/lifecycle", |_data| Ok(())).await.unwrap();
            wattle.publish("bench/lifecycle", DataFormat::text("test")).await.unwrap();
            
            tokio::time::sleep(Duration::from_millis(1)).await;
            
            wattle.unsubscribe(sub_id).await.unwrap();
            wattle.shutdown().await.unwrap();
            
            black_box(wattle);
        });
    });
    
    group.finish();
}

criterion_group!(
    benches,
    pubsub_benchmark,
    request_response_benchmark,
    concurrent_subscribers_benchmark,
    data_format_benchmark,
    memory_usage_benchmark
);

criterion_main!(benches);
