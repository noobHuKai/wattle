use async_trait::async_trait;
use std::collections::HashMap;
use crate::{Worker, Workflow, WattleError};
use crate::errors::{StorageError, RuntimeError, CoordinatorError};

/// 任务执行器 trait
#[async_trait]
pub trait TaskExecutor: Send + Sync {
    async fn execute<F1, F2, F3>(&self, 
        worker: Worker, 
        on_create: F1,
        on_success: F2, 
        on_fail: F3
    ) -> Result<(), RuntimeError>
    where
        F1: std::future::Future<Output = ()> + Send + 'static,
        F2: std::future::Future<Output = ()> + Send + 'static,
        F3: std::future::Future<Output = ()> + Send + 'static;
}

/// 存储仓库 trait
#[async_trait]
pub trait WorkflowRepository: Send + Sync {
    // 工作流操作
    async fn insert_workflow(&self, workflow: &Workflow) -> Result<(), StorageError>;
    async fn get_workflow(&self, name: &str) -> Result<Option<Workflow>, StorageError>;
    async fn list_workflows(&self) -> Result<Vec<Workflow>, StorageError>;
    async fn update_workflow_status(&self, name: &str, status: &str) -> Result<(), StorageError>;
    async fn delete_workflow(&self, name: &str) -> Result<(), StorageError>;
    
    // 分页查询
    async fn get_workflows_paged(
        &self, 
        limit: i64, 
        offset: i64, 
        status_filter: Option<String>,
        sort_by: Option<String>,
        order: Option<String>
    ) -> Result<(Vec<Workflow>, i64), StorageError>;
}

#[async_trait]
pub trait WorkerRepository: Send + Sync {
    // 工作者操作
    async fn insert_workers(&self, workers: &[Worker]) -> Result<(), StorageError>;
    async fn get_worker(&self, name: &str) -> Result<Option<Worker>, StorageError>;
    async fn list_workers_by_workflow(&self, workflow_name: &str) -> Result<Vec<Worker>, StorageError>;
    async fn update_worker_status(&self, name: &str, status: &str) -> Result<(), StorageError>;
    async fn delete_workers_by_workflow(&self, workflow_name: &str) -> Result<(), StorageError>;
    
    // 分页查询
    async fn get_workers_paged(
        &self,
        workflow_name: Option<String>,
        limit: i64,
        offset: i64,
        status_filter: Option<String>,
        sort_by: Option<String>,
        order: Option<String>
    ) -> Result<(Vec<Worker>, i64), StorageError>;
}

/// 综合存储库 trait
#[async_trait]
pub trait Repository: WorkflowRepository + WorkerRepository + Send + Sync {
    async fn health_check(&self) -> Result<(), StorageError>;
}

/// 协调器 trait
#[async_trait]
pub trait WorkflowCoordinator: Send + Sync {
    // 工作流管理
    async fn create_workflow(&self, workflow: &Workflow) -> Result<(), CoordinatorError>;
    async fn start_workflow(&self, workflow_name: &str) -> Result<(), CoordinatorError>;
    async fn stop_workflow(&self, workflow_name: &str) -> Result<(), CoordinatorError>;
    async fn get_workflow_status(&self, workflow_name: &str) -> Result<String, CoordinatorError>;
    
    // 工作者管理
    async fn run_worker(&self, worker_name: &str) -> Result<(), CoordinatorError>;
    async fn stop_worker(&self, worker_name: &str) -> Result<(), CoordinatorError>;
    async fn get_worker_status(&self, worker_name: &str) -> Result<String, CoordinatorError>;
    
    // 批量操作
    async fn run_workers(&self, worker_names: &[String]) -> Result<(), CoordinatorError>;
    
    // 查询操作
    async fn list_workflows(&self) -> Result<Vec<Workflow>, CoordinatorError>;
    async fn list_workers(&self, workflow_name: &str) -> Result<Vec<Worker>, CoordinatorError>;
    
    // 分页查询
    async fn get_workflows_paged(
        &self,
        limit: i64,
        offset: i64,
        status_filter: Option<String>,
        sort_by: Option<String>,
        order: Option<String>
    ) -> Result<(Vec<Workflow>, i64), CoordinatorError>;
}

/// 日志管理 trait
#[async_trait] 
pub trait LogManager: Send + Sync {
    async fn write_log(&self, worker_name: &str, workflow_name: &str, content: &str) -> Result<(), RuntimeError>;
    async fn read_logs(&self, worker_name: &str, workflow_name: &str, limit: Option<usize>) -> Result<Vec<String>, RuntimeError>;
    async fn stream_logs(&self, worker_name: &str, workflow_name: &str) -> Result<tokio::sync::mpsc::Receiver<String>, RuntimeError>;
}

/// 指标收集器 trait
#[async_trait]
pub trait MetricsCollector: Send + Sync {
    async fn record_workflow_started(&self, workflow_name: &str) -> Result<(), WattleError>;
    async fn record_workflow_completed(&self, workflow_name: &str, duration: std::time::Duration) -> Result<(), WattleError>;
    async fn record_workflow_failed(&self, workflow_name: &str, error: &str) -> Result<(), WattleError>;
    async fn record_worker_started(&self, worker_name: &str, workflow_name: &str) -> Result<(), WattleError>;
    async fn record_worker_completed(&self, worker_name: &str, workflow_name: &str, duration: std::time::Duration) -> Result<(), WattleError>;
    async fn record_worker_failed(&self, worker_name: &str, workflow_name: &str, error: &str) -> Result<(), WattleError>;
    async fn get_system_metrics(&self) -> Result<HashMap<String, f64>, WattleError>;
}

/// 事件发布器 trait（用于 SSE 优化）
#[async_trait]
pub trait EventPublisher: Send + Sync {
    async fn publish_workflow_event(&self, workflow_name: &str, event_type: &str, data: &str) -> Result<(), WattleError>;
    async fn publish_worker_event(&self, worker_name: &str, workflow_name: &str, event_type: &str, data: &str) -> Result<(), WattleError>;
    async fn publish_log_event(&self, worker_name: &str, workflow_name: &str, log_line: &str) -> Result<(), WattleError>;
    async fn subscribe_events(&self) -> Result<tokio::sync::broadcast::Receiver<EventMessage>, WattleError>;
}

/// 事件消息结构
#[derive(Debug, Clone)]
pub struct EventMessage {
    pub event_type: String,
    pub workflow_name: String,
    pub worker_name: Option<String>,
    pub data: String,
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

impl EventMessage {
    pub fn new(event_type: &str, workflow_name: &str, data: &str) -> Self {
        Self {
            event_type: event_type.to_string(),
            workflow_name: workflow_name.to_string(),
            worker_name: None,
            data: data.to_string(),
            timestamp: chrono::Utc::now(),
        }
    }

    pub fn with_worker(mut self, worker_name: &str) -> Self {
        self.worker_name = Some(worker_name.to_string());
        self
    }
}