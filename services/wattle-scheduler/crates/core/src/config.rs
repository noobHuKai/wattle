use std::{path::PathBuf, time::Duration};

use serde::Deserialize;

/// 定义任务执行的基本参数
#[derive(Debug, Clone, Deserialize)]
pub struct ExecutionConfig {
    pub log_dir: Option<PathBuf>,  // 日志存储目录
    pub timeout: Option<Duration>, // 默认超时时间
}
impl Default for ExecutionConfig {
    fn default() -> Self {
        Self {
            log_dir: Some(PathBuf::from("logs")),
            // timeout: Some(Duration::from_secs(3600)), // 1小时
            timeout: None,
        }
    }
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct SchedulerConfig {
    pub db_url: Option<String>,
    pub execution: Option<ExecutionConfig>,
}
