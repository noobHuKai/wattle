use serde::{Deserialize, Serialize};
use config::{Config, ConfigError, Environment, File};

/// 应用程序全局配置（支持分组，所有字段均为基本类型或 Option）
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Settings {
    #[serde(default)]
    pub server: ServerConfig,
    #[serde(default)]
    pub coordinator: CoordinatorConfig,
    #[serde(default)]
    pub database: DatabaseConfig,
    #[serde(default)]
    pub logging: LoggingConfig,
    #[serde(default)]
    pub execution: ExecutionConfig,
}

/// 服务器配置
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ServerConfig {
    pub host: Option<String>,
    pub port: Option<u16>,
    pub cors_origins: Option<String>, // 逗号分隔
    pub request_timeout_secs: Option<u64>,
}

/// 协调器配置
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CoordinatorConfig {
    pub mode: Option<String>,
    pub db_url: Option<String>,
    pub worker_timeout_secs: Option<u64>,
    pub max_concurrent_workers: Option<usize>,
}

/// 数据库配置
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DatabaseConfig {
    pub max_connections: Option<u32>,
    pub connection_timeout_secs: Option<u64>,
    pub idle_timeout_secs: Option<u64>,
}

/// 日志配置
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct LoggingConfig {
    pub level: Option<String>,
    pub file_path: Option<String>,
    pub max_file_size: Option<u64>,
    pub max_files: Option<usize>,
}

/// 执行配置
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ExecutionConfig {
    pub log_dir: Option<String>,
    pub timeout_secs: Option<u64>,
    pub max_parallel_tasks: Option<usize>,
}



impl Settings {
    /// 加载配置，优先级：环境变量 > 配置文件 > 默认值
    pub fn new() -> Result<Self, ConfigError> {
        let config = Config::builder()
            .add_source(File::with_name("configs/config").required(false))
            .add_source(Environment::with_prefix("WATTLE").prefix_separator("_").separator("__"))
            .build()?;
        let mut s: Settings = config.try_deserialize()?;
        s.apply_defaults();
        Ok(s)
    }

    pub fn from_file(path: &str) -> Result<Self, ConfigError> {
        let config = Config::builder()
            .add_source(File::with_name(path))
            .add_source(Environment::with_prefix("WATTLE").prefix_separator("_").separator("__"))
            .build()?;
        let mut s: Settings = config.try_deserialize()?;
        s.apply_defaults();
        Ok(s)
    }

    /// 补全所有默认值
    pub fn apply_defaults(&mut self) {
        // server defaults
        if self.server.host.is_none() { self.server.host = Some("localhost".to_string()); }
        if self.server.port.is_none() { self.server.port = Some(9240); }
        if self.server.cors_origins.is_none() { self.server.cors_origins = Some("*".to_string()); }
        if self.server.request_timeout_secs.is_none() { self.server.request_timeout_secs = Some(30); }

        // coordinator defaults
        if self.coordinator.mode.is_none() { self.coordinator.mode = Some("Sequential".to_string()); }
        if self.coordinator.db_url.is_none() { self.coordinator.db_url = Some("configs/wattle.db".to_string()); }
        if self.coordinator.worker_timeout_secs.is_none() { self.coordinator.worker_timeout_secs = Some(3600); }
        if self.coordinator.max_concurrent_workers.is_none() { self.coordinator.max_concurrent_workers = Some(100); }

        // database defaults
        if self.database.max_connections.is_none() { self.database.max_connections = Some(50); }
        if self.database.connection_timeout_secs.is_none() { self.database.connection_timeout_secs = Some(30); }
        if self.database.idle_timeout_secs.is_none() { self.database.idle_timeout_secs = Some(600); }

        // logging defaults
        if self.logging.level.is_none() { self.logging.level = Some("info".to_string()); }
        if self.logging.file_path.is_none() { self.logging.file_path = Some("logs/wattle.log".to_string()); }
        if self.logging.max_file_size.is_none() { self.logging.max_file_size = Some(10 * 1024 * 1024); }
        if self.logging.max_files.is_none() { self.logging.max_files = Some(5); }

        // execution defaults
        if self.execution.log_dir.is_none() { self.execution.log_dir = Some("logs".to_string()); }
        if self.execution.timeout_secs.is_none() { self.execution.timeout_secs = Some(0); }
        if self.execution.max_parallel_tasks.is_none() { self.execution.max_parallel_tasks = Some(10); }
    }
}

// 不再需要 Default trait，所有默认值由 apply_defaults 方法补全
