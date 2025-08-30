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
use zenoh::{Session, pubsub::Subscriber, query::Reply};

pub mod types;
pub mod client;
pub mod arrow_support;

pub use types::*;
pub use client::*;
pub use arrow_support::*;

#[allow(unused)]
pub struct Wattle {
    task_name: String,
    group_name: String,
    session: Session,
}

impl Wattle {
    pub async fn new() -> Result<Self> {
        let task_name = std::env::var("WATTLE_TASK_NAME").map_err(|_| eyre!("Must In Wattle"))?;
        let group_name = std::env::var("WATTLE_GROUP_NAME").map_err(|_| eyre!("Must In Wattle"))?;

        let session = zenoh::open(zenoh::Config::default())
            .await
            .map_err(|e| eyre!(e))?;

        Ok(Self {
            task_name,
            group_name,
            session,
        })
    }

    fn service_name(&self, name: &str) -> Result<String> {
        if name.contains("/") {
            bail!("{name} contain /");
        }
        let name = name.trim();
        Ok(format!("wattle/{}/{}", self.group_name, name))
    }
    pub async fn publish_json(&self, topic: &str, data: Value) -> Result<()> {
        self.session
            .put(self.service_name(topic)?, data.to_string())
            .await
            .map_err(|e| eyre!(e))?;

        Ok(())
    }

    pub async fn subscribe_json<T: Fn(Value) -> ()>(&self, topic: &str, callback: T) -> Result<()> {
        self.subscribe_raw(topic, |data| {
            let value = serde_json::from_slice(&data).unwrap_or(json!({}));
            callback(value)
        })
        .await?;
        Ok(())
    }

    pub async fn publish_raw(&self, topic: &str, data: &[u8]) -> Result<()> {
        self.session
            .put(self.service_name(topic)?, data)
            .await
            .map_err(|e| eyre!(e))?;
        Ok(())
    }

    pub async fn subscribe_raw<T: Fn(Cow<[u8]>) -> ()>(
        &self,
        topic: &str,
        callback: T,
    ) -> Result<()> {
        let subscriber = self
            .session
            .declare_subscriber(self.service_name(topic)?)
            .await
            .map_err(|e| eyre!(e))?;

        while let Ok(sample) = subscriber.recv_async().await {
            callback(sample.payload().to_bytes());
        }
        Ok(())
    }
}
