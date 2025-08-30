use std::{path::PathBuf, time::Duration};
use serde::{Deserialize, Serialize};
use config::{Config, ConfigError, Environment, File};

/// 应用程序全局配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Settings {
    pub server: ServerConfig,
    pub coordinator: CoordinatorConfig,
    pub execution: ExecutionConfig,
    pub logging: LoggingConfig,
}

/// 服务器配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerConfig {
    pub host: String,
    pub port: u16,
    pub cors_origins: Vec<String>,
    pub request_timeout: Duration,
}

/// 协调器配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinatorConfig {
    pub mode: String,
    pub db_url: String,
    pub database: DatabaseConfig,
    pub worker_timeout: Duration,
    pub max_concurrent_workers: usize,
}

/// 数据库配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DatabaseConfig {
    pub max_connections: Option<u32>,
    pub connection_timeout: Duration,
    pub idle_timeout: Duration,
}

/// 日志配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggingConfig {
    pub level: String,
    pub file_path: Option<PathBuf>,
    pub max_file_size: u64,
    pub max_files: usize,
}

/// 定义任务执行的基本参数
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionConfig {
    pub log_dir: Option<PathBuf>,  // 日志存储目录
    pub timeout: Option<Duration>, // 默认超时时间
    pub max_parallel_tasks: usize, // 最大并行任务数
}

impl Settings {
    /// 从多个配置源加载设置
    /// 优先级：环境变量 > 配置文件 > 默认值
    pub fn new() -> Result<Self, ConfigError> {
        let config = Config::builder()
            // 从默认值开始
            .add_source(Config::try_from(&Settings::default())?)
            // 从配置文件加载
            .add_source(File::with_name("configs/config").required(false))
            // 从环境变量加载（使用 WATTLE_ 前缀）
            .add_source(
                Environment::with_prefix("WATTLE")
                    .prefix_separator("_")
                    .separator("__")
            )
            .build()?;

        config.try_deserialize()
    }

    /// 从指定路径加载配置
    pub fn from_file(path: &str) -> Result<Self, ConfigError> {
        let config = Config::builder()
            .add_source(Config::try_from(&Settings::default())?)
            .add_source(File::with_name(path))
            .add_source(
                Environment::with_prefix("WATTLE")
                    .prefix_separator("_")
                    .separator("__")
            )
            .build()?;

        config.try_deserialize()
    }
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            server: ServerConfig::default(),
            coordinator: CoordinatorConfig::default(),
            execution: ExecutionConfig::default(),
            logging: LoggingConfig::default(),
        }
    }
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            host: "localhost".to_string(),
            port: 9240,
            cors_origins: vec!["*".to_string()],
            request_timeout: Duration::from_secs(30),
        }
    }
}

impl Default for CoordinatorConfig {
    fn default() -> Self {
        Self {
            mode: "Sequential".to_string(),
            db_url: "configs/wattle.db".to_string(),
            database: DatabaseConfig::default(),
            worker_timeout: Duration::from_secs(3600),
            max_concurrent_workers: 100,
        }
    }
}

impl Default for DatabaseConfig {
    fn default() -> Self {
        Self {
            max_connections: Some(50),
            connection_timeout: Duration::from_secs(30),
            idle_timeout: Duration::from_secs(600),
        }
    }
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            level: "info".to_string(),
            file_path: Some(PathBuf::from("logs/wattle.log")),
            max_file_size: 10 * 1024 * 1024, // 10MB
            max_files: 5,
        }
    }
}

impl Default for ExecutionConfig {
    fn default() -> Self {
        Self {
            log_dir: Some(PathBuf::from("logs")),
            timeout: None,
            max_parallel_tasks: 10,
        }
    }
}
