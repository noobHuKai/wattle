//! Wattle SDK 类型定义

use std::collections::HashMap;
use serde::{Deserialize, Serialize};

/// 消息类型
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum MessageType {
    /// 发布消息
    Publish,
    /// 请求消息
    Request,
    /// 回复消息
    Reply,
    /// 流式数据开始
    StreamStart,
    /// 流式数据块
    StreamChunk,
    /// 流式数据结束
    StreamEnd,
}

/// 数据格式
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DataFormat {
    /// JSON 格式
    Json,
    /// Apache Arrow 格式
    Arrow,
    /// 原始字节
    Raw,
}

/// Wattle 消息结构
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WattleMessage {
    /// 消息唯一标识
    pub id: String,
    /// 消息类型
    pub message_type: MessageType,
    /// 数据格式
    pub format: DataFormat,
    /// 消息数据
    pub data: Vec<u8>,
    /// 元数据
    pub metadata: HashMap<String, String>,
}

/// 流式传输会话
#[derive(Debug, Clone)]
pub struct StreamSession {
    pub session_id: String,
    pub total_chunks: Option<usize>,
    pub current_chunk: usize,
}

/// 请求选项
#[derive(Debug, Clone)]
pub struct RequestOptions {
    /// 超时时间（秒）
    pub timeout_secs: Option<u64>,
    /// 是否需要确认
    pub require_ack: bool,
    /// 重试次数
    pub retry_count: usize,
}

impl Default for RequestOptions {
    fn default() -> Self {
        Self {
            timeout_secs: Some(30),
            require_ack: false,
            retry_count: 0,
        }
    }
}

/// 订阅选项
#[derive(Debug, Clone)]
pub struct SubscribeOptions {
    /// 是否只接收最新消息
    pub latest_only: bool,
    /// 消息队列大小
    pub queue_size: usize,
}

impl Default for SubscribeOptions {
    fn default() -> Self {
        Self {
            latest_only: false,
            queue_size: 1000,
        }
    }
}
