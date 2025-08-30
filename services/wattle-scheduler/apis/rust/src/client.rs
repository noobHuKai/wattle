//! Wattle 客户端高级功能

use std::time::Duration;
use std::sync::Arc;
use eyre::{Result, eyre};
use serde_json::Value;
use uuid::Uuid;
use arrow_array::RecordBatch;
use tokio::time::timeout;
use futures::StreamExt;

use crate::types::*;
use crate::arrow_support::{arrow_to_bytes, bytes_to_arrow};
use crate::WattleClient;

impl WattleClient {
    /// 发送 JSON 请求并等待回复 - 完整实现
    pub async fn request_json(&self, target_worker: &str, service_name: &str, data: &Value, timeout_secs: Option<u64>) -> Result<Value> {
        let target_key = self.build_target_key(target_worker, service_name);
        let request_id = Uuid::new_v4().to_string();
        let reply_key = format!("{}/reply/{}", target_key, request_id);
        
        // 创建回复通道
        let (tx, rx) = tokio::sync::oneshot::channel();
        self.pending_requests.write().await.insert(request_id.clone(), tx);
        
        // 订阅回复
        let subscriber = self.session.declare_subscriber(&reply_key).await
            .map_err(|e| eyre!("Failed to create reply subscriber: {}", e))?;
        
        let pending_requests = self.pending_requests.clone();
        let reply_request_id = request_id.clone();
        
        // 启动回复监听器
        tokio::spawn(async move {
            let mut stream = subscriber.stream();
            if let Some(sample) = stream.next().await {
                if let Ok(reply_message) = serde_json::from_slice::<WattleMessage>(&sample.payload().to_bytes()) {
                    if let Some(tx) = pending_requests.write().await.remove(&reply_request_id) {
                        let _ = tx.send(reply_message);
                    }
                }
            }
        });
        
        // 发送请求
        let message = WattleMessage {
            id: request_id,
            message_type: MessageType::Request,
            format: DataFormat::Json,
            data: data.to_string().into_bytes(),
            metadata: std::collections::HashMap::from([
                ("reply_key".to_string(), reply_key),
            ]),
        };
        
        let serialized = serde_json::to_vec(&message)?;
        self.session.put(&target_key, serialized).await
            .map_err(|e| eyre!("Failed to send request: {}", e))?;
        
        // 等待回复
        let timeout_duration = Duration::from_secs(timeout_secs.unwrap_or(30));
        match timeout(timeout_duration, rx).await {
            Ok(Ok(reply_message)) => {
                if reply_message.format == DataFormat::Json {
                    let json_value: Value = serde_json::from_slice(&reply_message.data)?;
                    Ok(json_value)
                } else {
                    Err(eyre!("Expected JSON reply, got {:?}", reply_message.format))
                }
            },
            Ok(Err(_)) => Err(eyre!("Request channel was closed")),
            Err(_) => Err(eyre!("Request timeout after {} seconds", timeout_duration.as_secs())),
        }
    }

    /// 发送 Arrow 请求并等待回复
    pub async fn request_arrow(&self, target_worker: &str, service_name: &str, batch: &RecordBatch, timeout_secs: Option<u64>) -> Result<RecordBatch> {
        let target_key = self.build_target_key(target_worker, service_name);
        let request_id = Uuid::new_v4().to_string();
        let reply_key = format!("{}/reply/{}", target_key, request_id);
        let data = arrow_to_bytes(batch)?;
        
        // 创建回复通道
        let (tx, rx) = tokio::sync::oneshot::channel();
        self.pending_requests.write().await.insert(request_id.clone(), tx);
        
        // 订阅回复
        let subscriber = self.session.declare_subscriber(&reply_key).await
            .map_err(|e| eyre!("Failed to create reply subscriber: {}", e))?;
        
        let pending_requests = self.pending_requests.clone();
        let reply_request_id = request_id.clone();
        
        // 启动回复监听器
        tokio::spawn(async move {
            let mut stream = subscriber.stream();
            if let Some(sample) = stream.next().await {
                if let Ok(reply_message) = serde_json::from_slice::<WattleMessage>(&sample.payload().to_bytes()) {
                    if let Some(tx) = pending_requests.write().await.remove(&reply_request_id) {
                        let _ = tx.send(reply_message);
                    }
                }
            }
        });
        
        // 发送请求
        let message = WattleMessage {
            id: request_id,
            message_type: MessageType::Request,
            format: DataFormat::Arrow,
            data,
            metadata: std::collections::HashMap::from([
                ("reply_key".to_string(), reply_key),
            ]),
        };
        
        let serialized = serde_json::to_vec(&message)?;
        self.session.put(&target_key, serialized).await
            .map_err(|e| eyre!("Failed to send arrow request: {}", e))?;
        
        // 等待回复
        let timeout_duration = Duration::from_secs(timeout_secs.unwrap_or(30));
        match timeout(timeout_duration, rx).await {
            Ok(Ok(reply_message)) => {
                if reply_message.format == DataFormat::Arrow {
                    bytes_to_arrow(&reply_message.data)
                } else {
                    Err(eyre!("Expected Arrow reply, got {:?}", reply_message.format))
                }
            },
            Ok(Err(_)) => Err(eyre!("Request channel was closed")),
            Err(_) => Err(eyre!("Request timeout after {} seconds", timeout_duration.as_secs())),
        }
    }

    /// 订阅 JSON 数据
    pub async fn subscribe_json<F>(&self, service_name: &str, callback: F) -> Result<()>
    where
        F: Fn(Value) + Send + Sync + 'static,
    {
        let key = self.build_key(service_name);
        let subscriber = self.session.declare_subscriber(&key).await
            .map_err(|e| eyre!("Failed to create subscriber: {}", e))?;

        let callback = Arc::new(callback);
        
        tokio::spawn(async move {
            let mut stream = subscriber.stream();
            while let Some(sample) = stream.next().await {
                if let Ok(message) = serde_json::from_slice::<WattleMessage>(&sample.payload().to_bytes()) {
                    if message.format == DataFormat::Json {
                        if let Ok(json_value) = serde_json::from_slice::<Value>(&message.data) {
                            callback(json_value);
                        }
                    }
                }
            }
        });

        Ok(())
    }

    /// 订阅 Arrow 数据
    pub async fn subscribe_arrow<F>(&self, service_name: &str, callback: F) -> Result<()>
    where
        F: Fn(RecordBatch) + Send + Sync + 'static,
    {
        let key = self.build_key(service_name);
        let subscriber = self.session.declare_subscriber(&key).await
            .map_err(|e| eyre!("Failed to create subscriber: {}", e))?;

        let callback = Arc::new(callback);
        
        tokio::spawn(async move {
            let mut stream = subscriber.stream();
            while let Some(sample) = stream.next().await {
                if let Ok(message) = serde_json::from_slice::<WattleMessage>(&sample.payload().to_bytes()) {
                    if message.format == DataFormat::Arrow {
                        if let Ok(batch) = bytes_to_arrow(&message.data) {
                            callback(batch);
                        }
                    }
                }
            }
        });

        Ok(())
    }

    /// 发送请求并等待回复 (JSON) - 简化版本（向后兼容）
    pub async fn request_json_simple(&self, target_worker: &str, service_name: &str, data: &Value) -> Result<Value> {
        self.request_json(target_worker, service_name, data, None).await
    }

    /// 发送 Arrow 请求 - 简化版本（向后兼容）
    pub async fn request_arrow_simple(&self, target_worker: &str, service_name: &str, batch: &RecordBatch) -> Result<RecordBatch> {
        self.request_arrow(target_worker, service_name, batch, None).await
    }

    /// 构建目标服务的 key
    pub fn build_target_key(&self, target_worker: &str, service_name: &str) -> String {
        format!("wattle/services/{}/{}/{}", 
                self.workflow_name, target_worker, service_name)
    }

    /// 构建服务名称的 key
    pub fn build_key(&self, service_name: &str) -> String {
        format!("wattle/services/{}/{}/{}", 
                self.workflow_name, self.worker_name, service_name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arrow_support::create_sample_batch;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_request_with_timeout() {
        std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
        std::env::set_var("WATTLE_WORKER_NAME", "test_worker");
        
        let client = WattleClient::new().await.unwrap();
        let data = serde_json::json!({"test": "data"});
        
        // 测试超时（没有服务端响应）
        let result = client.request_json("target_worker", "test_service", &data, Some(1)).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("timeout"));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_subscription() {
        std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
        std::env::set_var("WATTLE_WORKER_NAME", "test_worker");
        
        let client = WattleClient::new().await.unwrap();
        
        let received = Arc::new(tokio::sync::RwLock::new(Vec::new()));
        let received_clone = received.clone();
        
        // 订阅数据
        client.subscribe_json("test_service", move |data| {
            let received = received_clone.clone();
            tokio::spawn(async move {
                received.write().await.push(data);
            });
        }).await.unwrap();
        
        // 等待一小会儿让订阅生效
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
        
        // 发布数据
        let test_data = serde_json::json!({"message": "hello"});
        client.publish_json("test_service", &test_data).await.unwrap();
        
        // 等待消息处理
        tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;
        
        // 验证接收到消息
        let received_messages = received.read().await;
        assert!(!received_messages.is_empty());
    }
}
