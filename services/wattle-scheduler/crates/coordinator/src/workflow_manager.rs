use core::{Worker, Workflow};
use eyre::Result;
use storage::{Repositories, WorkflowEntity};
use std::sync::Arc;

/// 工作流管理器
#[derive(Clone)]
pub struct WorkflowManager {
    repo: Arc<Repositories>,
}

impl WorkflowManager {
    pub fn new(repo: Arc<Repositories>) -> Self {
        Self { repo }
    }

    /// 检查工作流是否存在
    pub async fn workflow_exists(&self, name: &str) -> Result<bool> {
        self.repo.workflow_exists(name).await
    }

    /// 获取工作流
    pub async fn get_workflow(&self, name: &str) -> Result<Option<Workflow>> {
        if let Some(workflow_entity) = self.repo.get_workflow(name).await? {
            let mut workflow: Workflow = workflow_entity.into();
            let worker_entities = self.repo.get_workers_by_workflow(name).await?;
            let workers: Vec<Worker> = worker_entities
                .into_iter()
                .map(|worker_entity| worker_entity.into())
                .collect();

            workflow.workers = workers;
            Ok(Some(workflow))
        } else {
            Ok(None)
        }
    }

    /// 列出所有工作流
    pub async fn list_workflows(&self) -> Result<Vec<WorkflowEntity>> {
        self.repo.get_all_workflows().await
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
        self.repo
            .get_workflows_paged(
                page,
                page_size,
                status_filter.as_deref(),
                sort_by.as_deref().unwrap_or("created_at"),
                order.as_deref().unwrap_or("asc"),
            )
            .await
    }

    /// 获取工作流信息
    pub async fn get_workflow_info(&self, name: &str) -> Result<Option<WorkflowEntity>> {
        self.repo.get_workflow(name).await
    }

    /// 创建工作流
    pub async fn create_workflow(&self, workflow: Workflow) -> Result<()> {
        // 检查工作流是否已存在
        if self.repo.workflow_exists(&workflow.name).await? {
            return Err(eyre::eyre!("Workflow '{}' already exists", workflow.name));
        }

        // 插入工作流
        self.repo.insert_workflow(&workflow).await?;

        // 插入工作者
        if !workflow.workers.is_empty() {
            self.repo.insert_workers(&workflow.workers).await?;
        }

        Ok(())
    }

    /// 删除工作流
    pub async fn delete_workflow(&self, name: &str) -> Result<()> {
        if !self.repo.workflow_exists(name).await? {
            return Err(eyre::eyre!("Workflow '{}' not found", name));
        }

        self.repo.delete_workflow(name).await?;
        Ok(())
    }
}
