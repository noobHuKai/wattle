use crate::{WorkerInfo, zenoh_manager::ZenohManager, workflow_manager::WorkflowManager, worker_manager::WorkerManager};
use core::{
    CoordinatorConfig, ExecutionConfig, Settings,
    Workflow,
};
use eyre::{Result, eyre};
use runtime::TaskExecutor;
use storage::{Repositories, WorkerEntity, WorkflowEntity};
use std::{collections::HashMap, sync::Arc};
use tokio::sync::RwLock;

#[derive(Clone)]
pub struct Coordinator {
    repo: Arc<Repositories>,
    session: zenoh::Session,
    workflow_manager: WorkflowManager,
    worker_manager: WorkerManager,
}

impl Coordinator {
    pub async fn new(cfg: CoordinatorConfig, exec_cfg: ExecutionConfig) -> Result<Self> {
        let db_url = cfg.db_url.clone();
        let db_config = cfg.database.clone();
        let repo = Arc::new(Repositories::new(Some(db_url), Some(db_config)).await?);
        let executor = TaskExecutor::new(exec_cfg);

        let session = zenoh::open(zenoh::Config::default())
            .await
            .map_err(|e| eyre!(e))?;

        if let Err(e) = ZenohManager::run_zenohd().await {
            tracing::error!("Failed to start zenohd: {}", e);
        }

        let workflow_manager = WorkflowManager::new(repo.clone());
        let worker_manager = WorkerManager::new(repo.clone(), executor);

        Ok(Coordinator {
            repo,
            session,
            workflow_manager,
            worker_manager,
        })
    }

    /// 从完整设置创建协调器
    pub async fn from_settings(settings: Settings) -> Result<Self> {
        Self::new(settings.coordinator, settings.execution).await
    }

    /// 获取 PID 映射
    pub fn get_pid_maps(&self) -> Arc<RwLock<HashMap<u32, (String, String)>>> {
        self.worker_manager.get_pid_maps()
    }

    // ========== 工作流管理方法 ==========

    /// 检查工作流是否存在
    pub async fn workflow_exists(&self, name: &str) -> Result<bool> {
        self.workflow_manager.workflow_exists(name).await
    }

    /// 获取工作流
    pub async fn get_workflow(&self, name: &str) -> Result<Option<Workflow>> {
        self.workflow_manager.get_workflow(name).await
    }

    /// 列出所有工作流
    pub async fn list_workflows(&self) -> Result<Vec<WorkflowEntity>> {
        self.workflow_manager.list_workflows().await
    }

    /// 分页列出工作流
    pub async fn list_workflows_paged(
        &self,
        page: u64,
        page_size: u64,
        status_filter: Option<String>,
        sort_by: Option<String>,
        order: Option<String>,
    ) -> Result<(Vec<WorkflowEntity>, u64)> {
        self.workflow_manager
            .list_workflows_paged(page, page_size, status_filter, sort_by, order)
            .await
    }

    /// 获取工作流信息
    pub async fn get_workflow_info(&self, name: &str) -> Result<Option<WorkflowEntity>> {
        self.workflow_manager.get_workflow_info(name).await
    }

    /// 创建工作流
    pub async fn create_workflow(&self, workflow: Workflow) -> Result<()> {
        self.workflow_manager.create_workflow(workflow).await
    }

    // ========== 工作者管理方法 ==========

    /// 检查工作者是否存在
    pub async fn worker_exists(&self, workflow_name: &str, worker_name: &str) -> Result<bool> {
        self.worker_manager.worker_exists(workflow_name, worker_name).await
    }

    /// 列出工作者
    pub async fn list_workers(&self, workflow_name: &str) -> Result<Vec<WorkerEntity>> {
        self.worker_manager.list_workers(workflow_name).await
    }

    /// 运行工作流
    pub async fn run_workflow(&self, workflow_name: &String) -> Result<()> {
        self.worker_manager.run_workflow(workflow_name).await
    }

    /// 获取工作者信息
    pub async fn get_worker_info(&self, workflow_name: &str, worker_name: &str) -> Result<WorkerInfo> {
        self.worker_manager.get_worker_info(workflow_name, worker_name).await
    }

    /// 启动工作者
    pub async fn start_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        self.worker_manager.start_worker(workflow_name, worker_name).await
    }

    /// 停止工作者
    pub async fn stop_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        self.worker_manager.stop_worker(workflow_name, worker_name).await
    }

    /// 删除工作者
    pub async fn delete_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        self.worker_manager.delete_worker(workflow_name, worker_name).await
    }

    /// 获取工作者日志
    pub async fn get_worker_logs(&self, workflow_name: &str, worker_name: &str) -> Result<Vec<storage::WorkerLogEntity>> {
        self.repo.get_worker_logs(workflow_name, worker_name).await
    }

    // ========== Zenoh 管理方法 ==========

    /// 列出 Zenoh 主题
    pub async fn list_topic(&self, name: Option<String>) -> Result<Vec<String>> {
        ZenohManager::list_topics(&self.session, name).await
    }

    // ========== 仓库访问 ==========

    /// 获取仓库实例
    pub fn service(&self) -> &Repositories {
        &self.repo
    }
}
