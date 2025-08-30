//! Wattle Rust SDK
//! 
//! 提供 Worker 间通信的 Rust SDK，支持 Zenoh + Arrow 进行数据传输

use std::collections::HashMap;
use std::sync::Arc;

use eyre::{Result, bail, eyre};
use futures::StreamExt;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use tokio::sync::{RwLock, oneshot};
use uuid::Uuid;
use zenoh::{Session, pubsub::Subscriber};

pub mod types;
pub mod client;
pub mod arrow_support;

pub use types::*;
pub use client::*;
pub use arrow_support::*;

/// Wattle SDK 主结构体
pub struct WattleClient {
    workflow_name: String,
    worker_name: String,
    session: Arc<Session>,
    subscribers: Arc<RwLock<HashMap<String, Subscriber<'static>>>>,
    pending_requests: Arc<RwLock<HashMap<String, oneshot::Sender<WattleMessage>>>>
}

impl WattleClient {
    /// 创建新的 Wattle 客户端
    pub async fn new() -> Result<Self> {
        let workflow_name = std::env::var("WATTLE_WORKFLOW_NAME")
            .map_err(|_| eyre!("WATTLE_WORKFLOW_NAME environment variable not set"))?;
        let worker_name = std::env::var("WATTLE_WORKER_NAME")
            .map_err(|_| eyre!("WATTLE_WORKER_NAME environment variable not set"))?;

        let session = Arc::new(zenoh::open(zenoh::Config::default())
            .await
            .map_err(|e| eyre!("Failed to open Zenoh session: {}", e))?);

        Ok(Self {
            workflow_name,
            worker_name,
            session,
            subscribers: Arc::new(RwLock::new(HashMap::new())),
            pending_requests: Arc::new(RwLock::new(HashMap::new()))
        })
    }

    /// 构建服务名称的 key
    fn build_key(&self, service_name: &str) -> String {
        format!("wattle/services/{}/{}/{}", 
                self.workflow_name, self.worker_name, service_name)
    }

    /// 构建目标服务的 key
    fn build_target_key(&self, target_worker: &str, service_name: &str) -> String {
        format!("wattle/services/{}/{}/{}", 
                self.workflow_name, target_worker, service_name)
    }

    /// 发布 JSON 数据
    pub async fn publish_json(&self, service_name: &str, data: &Value) -> Result<()> {
        let key = self.build_key(service_name);
        let message = WattleMessage {
            id: Uuid::new_v4().to_string(),
            message_type: MessageType::Publish,
            format: DataFormat::Json,
            data: data.to_string().into_bytes(),
            metadata: HashMap::new(),
        };
        
        let serialized = serde_json::to_vec(&message)?;
        self.session.put(&key, serialized).await
            .map_err(|e| eyre!("Failed to publish: {}", e))?;
        
        Ok(())
    }

    /// 发布 Arrow 数据
    pub async fn publish_arrow(&self, service_name: &str, batch: &arrow::record_batch::RecordBatch) -> Result<()> {
        let key = self.build_key(service_name);
        let data = arrow_to_bytes(batch)?;
        let message = WattleMessage {
            id: Uuid::new_v4().to_string(),
            message_type: MessageType::Publish,
            format: DataFormat::Arrow,
            data,
            metadata: HashMap::new(),
        };
        
        let serialized = serde_json::to_vec(&message)?;
        self.session.put(&key, serialized).await
            .map_err(|e| eyre!("Failed to publish arrow: {}", e))?;
        
        Ok(())
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
        let mut stream = subscriber.stream();
        
        tokio::spawn(async move {
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

        self.subscribers.write().await.insert(service_name.to_string(), subscriber);
        Ok(())
    }

    /// 关闭客户端
    pub async fn close(&self) -> Result<()> {
        // 清理订阅者
        let mut subscribers = self.subscribers.write().await;
        subscribers.clear();
        
        // 关闭会话
        self.session.close().await
            .map_err(|e| eyre!("Failed to close session: {}", e))?;
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::time::{sleep, Duration};

    #[tokio::test]
    async fn test_client_creation() {
        std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
        std::env::set_var("WATTLE_WORKER_NAME", "test_worker");
        
        let result = WattleClient::new().await;
        assert!(result.is_ok());
    }

    #[tokio::test]
    async fn test_key_building() {
        std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
        std::env::set_var("WATTLE_WORKER_NAME", "test_worker");
        
        let client = WattleClient::new().await.unwrap();
        let key = client.build_key("test_service");
        assert_eq!(key, "wattle/services/test_workflow/test_worker/test_service");
    }
}
