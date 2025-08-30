use crate::WorkerInfo;
use core::{Worker, WorkerStatus};
use eyre::Result;
use runtime::TaskExecutor;
use storage::{Repositories, WorkerEntity};
use std::{collections::HashMap, sync::Arc};
use tokio::sync::RwLock;

/// 工作者管理器
#[derive(Clone)]
pub struct WorkerManager {
    repo: Arc<Repositories>,
    executor: TaskExecutor,
}

impl WorkerManager {
    pub fn new(repo: Arc<Repositories>, executor: TaskExecutor) -> Self {
        Self { repo, executor }
    }

    /// 检查工作者是否存在
    pub async fn worker_exists(&self, workflow_name: &str, worker_name: &str) -> Result<bool> {
        self.repo.worker_exists(workflow_name, worker_name).await
    }

    /// 列出工作者
    pub async fn list_workers(&self, workflow_name: &str) -> Result<Vec<WorkerEntity>> {
        self.repo.get_workers_by_workflow(workflow_name).await
    }

    /// 运行工作流
    pub async fn run_workflow(&self, workflow_name: &str) -> Result<()> {
        let workers = self.repo.get_workers_by_workflow(workflow_name).await?;

        if workers.is_empty() {
            return Err(eyre::eyre!("No workers found for workflow '{}'", workflow_name));
        }

        for worker_entity in workers {
            let worker: Worker = worker_entity.into();
            self.run_workers(worker).await?;
        }

        Ok(())
    }

    /// 运行单个工作者
    async fn run_workers(&self, worker: Worker) -> Result<()> {
        let workflow_name = worker.workflow_name.clone();
        let worker_name = worker.name.clone();

        // 更新工作者状态为运行中
        self.repo
            .update_worker_status(&workflow_name, &worker_name, WorkerStatus::Running)
            .await?;

        // 执行任务
        match self.executor.execute(
            worker,
            async {},  // on_create callback
            async {},  // on_success callback  
            async {},  // on_fail callback
        ).await {
            Ok(_) => {
                // 任务成功完成
                self.repo
                    .update_worker_status(&workflow_name, &worker_name, WorkerStatus::Completed)
                    .await?;
            }
            Err(e) => {
                // 任务执行失败
                self.repo
                    .update_worker_error(&workflow_name, &worker_name, &e.to_string())
                    .await?;
                return Err(e);
            }
        }

        Ok(())
    }

    /// 获取工作者信息
    pub async fn get_worker_info(&self, workflow_name: &str, worker_name: &str) -> Result<WorkerInfo> {
        let worker_entity = self
            .repo
            .get_workers_by_workflow(workflow_name)
            .await?
            .into_iter()
            .find(|w| w.name == worker_name)
            .ok_or_else(|| eyre::eyre!("Worker '{}' not found in workflow '{}'", worker_name, workflow_name))?;

        Ok(WorkerInfo {
            name: worker_entity.name,
            workflow_name: worker_entity.workflow_name,
            command: worker_entity.command,
            args: worker_entity.args.and_then(|args| serde_json::from_str(&args).ok()),
            working_dir: worker_entity.working_dir,
            env_vars: worker_entity.env_vars.and_then(|env| serde_json::from_str(&env).ok()),
            status: worker_entity.status,
        })
    }

    /// 启动工作者
    pub async fn start_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        if let Some(worker) = self.repo.get_worker(workflow_name, worker_name).await? {
            self.run_workers(worker).await?;
        }
        Ok(())
    }

    /// 停止工作者
    pub async fn stop_worker(&self, _workflow_name: &str, _worker_name: &str) -> Result<()> {
        // TODO: 实现停止工作者的逻辑
        tracing::warn!("Stop worker functionality not yet implemented");
        Ok(())
    }

    /// 删除工作者
    pub async fn delete_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        if !self.repo.worker_exists(workflow_name, worker_name).await? {
            return Err(eyre::eyre!("Worker '{}' not found in workflow '{}'", worker_name, workflow_name));
        }

        self.repo.delete_worker(workflow_name, worker_name).await?;
        Ok(())
    }

    /// 获取 PID 映射
    pub fn get_pid_maps(&self) -> Arc<RwLock<HashMap<u32, (String, String)>>> {
        self.executor.pid_maps.clone()
    }
}
