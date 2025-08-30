use thiserror::Error;
use std::fmt;

/// 系统错误类型
#[derive(Error, Debug)]
pub enum WattleError {
    #[error("Storage error: {0}")]
    Storage(#[from] StorageError),

    #[error("Coordinator error: {0}")]
    Coordinator(#[from] CoordinatorError),

    #[error("Runtime error: {0}")]
    Runtime(#[from] RuntimeError),

    #[error("Configuration error: {0}")]
    Config(#[from] config::ConfigError),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

/// 存储层错误
#[derive(Error, Debug)]
pub enum StorageError {
    #[error("Database error: {0}")]
    Database(#[from] sqlx::Error),

    #[error("Workflow '{0}' not found")]
    WorkflowNotFound(String),

    #[error("Worker '{0}' not found")]
    WorkerNotFound(String),

    #[error("Invalid query parameter: {0}")]
    InvalidQuery(String),

    #[error("Connection pool error: {0}")]
    ConnectionPool(String),

    #[error("Migration error: {0}")]
    Migration(String),
}

/// 协调器错误
#[derive(Error, Debug)]
pub enum CoordinatorError {
    #[error("Workflow '{0}' already exists")]
    WorkflowExists(String),

    #[error("Workflow '{0}' is not in runnable state: {1}")]
    WorkflowNotRunnable(String, String),

    #[error("Worker '{0}' is already running")]
    WorkerAlreadyRunning(String),

    #[error("Dependency cycle detected in workflow '{0}'")]
    DependencyCycle(String),

    #[error("Maximum concurrent workers exceeded: {0}/{1}")]
    MaxWorkersExceeded(usize, usize),

    #[error("Worker timeout: {0}")]
    WorkerTimeout(String),

    #[error("Invalid workflow configuration: {0}")]
    InvalidWorkflowConfig(String),
}

/// 运行时错误
#[derive(Error, Debug)]
pub enum RuntimeError {
    #[error("Task execution failed: {0}")]
    ExecutionFailed(String),

    #[error("Command not found: {0}")]
    CommandNotFound(String),

    #[error("Permission denied: {0}")]
    PermissionDenied(String),

    #[error("Resource exhausted: {0}")]
    ResourceExhausted(String),

    #[error("Timeout after {0} seconds")]
    Timeout(u64),

    #[error("Log write error: {0}")]
    LogWrite(String),
}

/// HTTP API 错误响应
#[derive(serde::Serialize)]
pub struct ErrorResponse {
    pub error: String,
    pub code: String,
    pub details: Option<String>,
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

impl ErrorResponse {
    pub fn new(error: impl fmt::Display, code: &str) -> Self {
        Self {
            error: error.to_string(),
            code: code.to_string(),
            details: None,
            timestamp: chrono::Utc::now(),
        }
    }

    pub fn with_details(mut self, details: impl fmt::Display) -> Self {
        self.details = Some(details.to_string());
        self
    }
}

/// 将内部错误映射到 HTTP 状态码和错误响应
impl WattleError {
    pub fn to_http_response(&self) -> (axum::http::StatusCode, ErrorResponse) {
        use axum::http::StatusCode;

        match self {
            WattleError::Storage(err) => match err {
                StorageError::WorkflowNotFound(name) => (
                    StatusCode::NOT_FOUND,
                    ErrorResponse::new(format!("Workflow '{}' not found", name), "WORKFLOW_NOT_FOUND")
                ),
                StorageError::WorkerNotFound(name) => (
                    StatusCode::NOT_FOUND,
                    ErrorResponse::new(format!("Worker '{}' not found", name), "WORKER_NOT_FOUND")
                ),
                StorageError::InvalidQuery(msg) => (
                    StatusCode::BAD_REQUEST,
                    ErrorResponse::new("Invalid query parameter", "INVALID_QUERY").with_details(msg)
                ),
                StorageError::Database(_) => (
                    StatusCode::INTERNAL_SERVER_ERROR,
                    ErrorResponse::new("Database error", "DATABASE_ERROR")
                ),
                _ => (
                    StatusCode::INTERNAL_SERVER_ERROR,
                    ErrorResponse::new("Storage error", "STORAGE_ERROR")
                ),
            },

            WattleError::Coordinator(err) => match err {
                CoordinatorError::WorkflowExists(name) => (
                    StatusCode::CONFLICT,
                    ErrorResponse::new(format!("Workflow '{}' already exists", name), "WORKFLOW_EXISTS")
                ),
                CoordinatorError::WorkflowNotRunnable(name, state) => (
                    StatusCode::CONFLICT,
                    ErrorResponse::new(
                        format!("Workflow '{}' is not runnable", name), 
                        "WORKFLOW_NOT_RUNNABLE"
                    ).with_details(format!("Current state: {}", state))
                ),
                CoordinatorError::WorkerAlreadyRunning(name) => (
                    StatusCode::CONFLICT,
                    ErrorResponse::new(format!("Worker '{}' is already running", name), "WORKER_RUNNING")
                ),
                CoordinatorError::DependencyCycle(name) => (
                    StatusCode::BAD_REQUEST,
                    ErrorResponse::new(format!("Dependency cycle in workflow '{}'", name), "DEPENDENCY_CYCLE")
                ),
                CoordinatorError::MaxWorkersExceeded(current, max) => (
                    StatusCode::TOO_MANY_REQUESTS,
                    ErrorResponse::new("Too many concurrent workers", "MAX_WORKERS_EXCEEDED")
                        .with_details(format!("{}/{} workers", current, max))
                ),
                CoordinatorError::InvalidWorkflowConfig(msg) => (
                    StatusCode::BAD_REQUEST,
                    ErrorResponse::new("Invalid workflow configuration", "INVALID_WORKFLOW_CONFIG")
                        .with_details(msg)
                ),
                _ => (
                    StatusCode::INTERNAL_SERVER_ERROR,
                    ErrorResponse::new("Coordinator error", "COORDINATOR_ERROR")
                ),
            },

            WattleError::Runtime(err) => match err {
                RuntimeError::CommandNotFound(cmd) => (
                    StatusCode::BAD_REQUEST,
                    ErrorResponse::new(format!("Command '{}' not found", cmd), "COMMAND_NOT_FOUND")
                ),
                RuntimeError::PermissionDenied(resource) => (
                    StatusCode::FORBIDDEN,
                    ErrorResponse::new("Permission denied", "PERMISSION_DENIED").with_details(resource)
                ),
                RuntimeError::Timeout(seconds) => (
                    StatusCode::REQUEST_TIMEOUT,
                    ErrorResponse::new(format!("Operation timeout after {} seconds", seconds), "TIMEOUT")
                ),
                _ => (
                    StatusCode::INTERNAL_SERVER_ERROR,
                    ErrorResponse::new("Runtime error", "RUNTIME_ERROR")
                ),
            },

            WattleError::Config(_) => (
                StatusCode::INTERNAL_SERVER_ERROR,
                ErrorResponse::new("Configuration error", "CONFIG_ERROR")
            ),

            WattleError::Io(_) => (
                StatusCode::INTERNAL_SERVER_ERROR,
                ErrorResponse::new("IO error", "IO_ERROR")
            ),
        }
    }
}
