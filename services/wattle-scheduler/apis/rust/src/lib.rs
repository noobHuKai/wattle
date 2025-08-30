//! Wattle Rust SDK
//! 
//! 提供 Worker 间通信的 Rust SDK，支持 Zenoh + Arrow 进行数据传输

use std::collections::HashMap;
use std::sync::Arc;

use eyre::{Result, eyre};
use serde_json::Value;
use tokio::sync::RwLock;
use uuid::Uuid;
use zenoh::Session;

pub mod types;
pub mod client;
pub mod arrow_support;

pub use types::*;
pub use arrow_support::create_sample_batch;
use arrow_support::{arrow_to_bytes};

/// Wattle SDK 主结构体
#[derive(Clone)]
pub struct WattleClient {
    pub session: Arc<Session>,
    pub workflow_name: String,
    pub worker_name: String,
    request_handlers: Arc<RwLock<HashMap<String, tokio::task::JoinHandle<()>>>>,
    pending_requests: Arc<RwLock<HashMap<String, tokio::sync::oneshot::Sender<WattleMessage>>>>,
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
            request_handlers: Arc::new(RwLock::new(HashMap::new())),
            pending_requests: Arc::new(RwLock::new(HashMap::new())),
        })
    }

    /// 发布 JSON 数据
    pub async fn publish_json(&self, service_name: &str, data: &Value) -> Result<()> {
        let key = format!("wattle/services/{}/{}/{}", 
                         self.workflow_name, self.worker_name, service_name);
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
    pub async fn publish_arrow(&self, service_name: &str, batch: &arrow_array::RecordBatch) -> Result<()> {
        let key = format!("wattle/services/{}/{}/{}", 
                         self.workflow_name, self.worker_name, service_name);
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

    /// 关闭客户端
    pub async fn close(&self) -> Result<()> {
        // 清理 request handlers
        let mut handlers = self.request_handlers.write().await;
        for (_, handle) in handlers.drain() {
            handle.abort();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_client_creation() {
        std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
        std::env::set_var("WATTLE_WORKER_NAME", "test_worker");
        
        let result = WattleClient::new().await;
        assert!(result.is_ok());
    }

    #[tokio::test]
    async fn test_json_publish() {
        std::env::set_var("WATTLE_WORKFLOW_NAME", "test_workflow");
        std::env::set_var("WATTLE_WORKER_NAME", "test_worker");
        
        let client = WattleClient::new().await.unwrap();
        let data = serde_json::json!({"test": "data"});
        
        let result = client.publish_json("test_service", &data).await;
        assert!(result.is_ok());
    }
}
