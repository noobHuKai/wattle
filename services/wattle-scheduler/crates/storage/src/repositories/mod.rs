mod workflow_repository;
mod worker_repository;
mod log_repository;

pub use workflow_repository::WorkflowRepository;
pub use worker_repository::WorkerRepository;
pub use log_repository::LogRepository;

use crate::{
    DB,
    model::{WorkerEntity, WorkflowEntity, WorkerLogEntity},
};
use core::{Worker, Workflow, WorkerStatus, DatabaseConfig};
use eyre::Result;

pub struct Repositories {
    db: DB,
}

impl Repositories {
    pub async fn new(db_url: Option<String>, db_config: Option<DatabaseConfig>) -> Result<Self> {
        let db = crate::init_database(db_url, db_config).await?;
        Ok(Self { db })
    }

    /// 获取工作流仓库
    pub fn workflows(&self) -> WorkflowRepository {
        WorkflowRepository::new(&self.db)
    }

    /// 获取工作者仓库
    pub fn workers(&self) -> WorkerRepository {
        WorkerRepository::new(&self.db)
    }

    /// 获取日志仓库
    pub fn logs(&self) -> LogRepository {
        LogRepository::new(&self.db)
    }

    // ========== 保持向后兼容的便捷方法 ==========

    /// 插入工作流
    pub async fn insert_workflow(&self, workflow: &Workflow) -> Result<()> {
        self.workflows().insert_workflow(workflow).await
    }

    /// 根据名称获取工作流
    pub async fn get_workflow(&self, name: &str) -> Result<Option<WorkflowEntity>> {
        self.workflows().get_workflow(name).await
    }

    /// 获取所有工作流
    pub async fn get_all_workflows(&self) -> Result<Vec<WorkflowEntity>> {
        self.workflows().get_all_workflows().await
    }

    /// 获取工作流列表（支持分页和过滤）
    pub async fn get_workflows_paged(
        &self,
        page: u64,
        page_size: u64,
        status_filter: Option<&str>,
        sort_by: &str,
        order: &str,
    ) -> Result<(Vec<WorkflowEntity>, u64)> {
        self.workflows().get_workflows_paged(page, page_size, status_filter, sort_by, order).await
    }

    /// 检查工作流是否存在
    pub async fn workflow_exists(&self, name: &str) -> Result<bool> {
        self.workflows().workflow_exists(name).await
    }

    /// 更新工作流状态
    pub async fn update_workflow_status(&self, name: &str, status: WorkerStatus) -> Result<()> {
        self.workflows().update_workflow_status(name, status).await
    }

    /// 删除工作流 (软删除)
    pub async fn delete_workflow(&self, name: &str) -> Result<()> {
        self.workflows().delete_workflow(name).await
    }

    /// 插入工作者
    pub async fn insert_workers(&self, workers: &[Worker]) -> Result<()> {
        self.workers().insert_workers(workers).await
    }

    /// 根据工作流名称和工作者名称获取工作者
    pub async fn get_worker(&self, workflow_name: &str, worker_name: &str) -> Result<Option<Worker>> {
        self.workers().get_worker(workflow_name, worker_name).await
    }

    /// 根据工作流名称获取所有工作者
    pub async fn get_workers_by_workflow(&self, workflow_name: &str) -> Result<Vec<WorkerEntity>> {
        self.workers().get_workers_by_workflow(workflow_name).await
    }

    /// 检查工作者是否存在
    pub async fn worker_exists(&self, workflow_name: &str, worker_name: &str) -> Result<bool> {
        self.workers().worker_exists(workflow_name, worker_name).await
    }

    /// 更新工作者状态
    pub async fn update_worker_status(
        &self,
        workflow_name: &str,
        worker_name: &str,
        status: WorkerStatus,
    ) -> Result<()> {
        self.workers().update_worker_status(workflow_name, worker_name, status).await
    }

    /// 更新工作者错误信息
    pub async fn update_worker_error(
        &self,
        workflow_name: &str,
        worker_name: &str,
        error_message: &str,
    ) -> Result<()> {
        self.workers().update_worker_error(workflow_name, worker_name, error_message).await
    }

    /// 删除工作者 (软删除)
    pub async fn delete_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        self.workers().delete_worker(workflow_name, worker_name).await
    }

    /// 插入工作者日志
    pub async fn insert_worker_log(
        &self,
        workflow_name: &str,
        worker_name: &str,
        log_type: &str,
        file_path: &str,
    ) -> Result<()> {
        self.logs().insert_worker_log(workflow_name, worker_name, log_type, file_path).await
    }

    /// 获取工作者日志
    pub async fn get_worker_logs(
        &self,
        workflow_name: &str,
        worker_name: &str,
    ) -> Result<Vec<WorkerLogEntity>> {
        self.logs().get_worker_logs(workflow_name, worker_name).await
    }
}
