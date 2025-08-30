use crate::WorkerInfo;
use core::{CoordinatorConfig, Worker, Workflow, WorkerStatus};
use std::{collections::HashMap, process::Command, sync::Arc};

use eyre::{Result, eyre};
use rand::Rng as _;
use runtime::TaskExecutor;
use storage::{Repositories, WorkerEntity, WorkflowEntity};
use tokio::{net::TcpListener, sync::RwLock};
use tracing::{error, info};

#[derive(Clone)]
pub struct Coordinator {
    repo: Arc<Repositories>,
    executor: TaskExecutor,
    session: zenoh::Session,
}

impl Coordinator {
    pub async fn new(cfg: CoordinatorConfig) -> Result<Self> {
        let db_url = cfg.db_url.clone();
        let db_config = cfg.database.clone();
        let repo = Arc::new(Repositories::new(db_url, db_config).await?);
        let executor = TaskExecutor::new(cfg.execution.unwrap_or_default());

        let session = zenoh::open(zenoh::Config::default())
            .await
            .map_err(|e| eyre!(e))?;

        if let Err(e) = Self::run_zenohd().await {
            error!("Failed to start zenohd: {}", e);
        }

        Ok(Coordinator {
            repo,
            executor,
            session,
        })
    }

    pub fn get_pid_maps(&self) -> Arc<RwLock<HashMap<u32, (String, String)>>> {
        self.executor.pid_maps.clone()
    }
    async fn run_zenohd() -> eyre::Result<()> {
        if Self::is_zenohd_running()? {
            info!("zenohd is already running.");
        } else {
            info!("zenohd not running, starting it...");
            let cmd = which::which("zenohd")?;
            let port = Self::get_free_port_in_range()
                .await
                .ok_or_else(|| eyre::eyre!("No free port found in range 20000-30000"))?;

            tokio::spawn(async move {
                let result = tokio::task::spawn_blocking(move || {
                    let child = Command::new(cmd)
                        .arg(format!("--rest-http-port={}", port))
                        .spawn();
                    match child {
                        Ok(mut child_proc) => match child_proc.wait() {
                            Ok(_) => {}
                            Err(e) => {
                                error!("Failed to wait for zenohd process: {}", e);
                            }
                        },
                        Err(e) => {
                            error!("Failed to spawn zenohd: {}", e);
                        }
                    }
                });
                // Await the blocking task to complete
                let _ = result.await;
            });
        }
        Ok(())
    }

    async fn get_free_port_in_range() -> Option<u16> {
        let mut rng = rand::rng();

        for _ in 0..50 {
            let port: u16 = rng.random_range(20000..=30000);
            if TcpListener::bind(("127.0.0.1", port)).await.is_ok() {
                return Some(port);
            }
        }
        None
    }
    /// 检查 zenohd 是否运行（跨平台版本）
    fn is_zenohd_running() -> eyre::Result<bool> {
        #[cfg(target_os = "linux")]
        {
            let output = Command::new("pgrep").arg("zenohd").output()?;
            Ok(!output.stdout.is_empty())
        }

        #[cfg(target_os = "macos")]
        {
            let output = Command::new("pgrep").arg("zenohd").output()?;
            Ok(!output.stdout.is_empty())
        }

        #[cfg(target_os = "windows")]
        {
            let output = Command::new("tasklist").output()?;
            Ok(String::from_utf8_lossy(&output.stdout).contains("zenohd.exe"))
        }
    }

    async fn run_workers(&self, worker: Worker) -> Result<()> {
        let workflow_name = worker.workflow_name.clone();
        let worker_name = worker.name.clone();
        
        // 使用引用计数减少克隆
        let repo = self.repo.clone();
        
        self.executor
            .execute(
                worker,
                {
                    let repo = repo.clone();
                    let workflow_name = workflow_name.clone();
                    let worker_name = worker_name.clone();
                    async move {
                        // 工作者创建时的逻辑
                        if let Err(err) = repo
                            .update_worker_status(&workflow_name, &worker_name, WorkerStatus::Running)
                            .await
                        {
                            tracing::error!("Failed to update worker status: {:?}", err);
                        };
                    }
                },
                {
                    let repo = repo.clone();
                    let workflow_name = workflow_name.clone();
                    let worker_name = worker_name.clone();
                    async move {
                        // 工作者成功时的逻辑
                        if let Err(err) = repo
                            .update_worker_status(&workflow_name, &worker_name, WorkerStatus::Completed)
                            .await
                        {
                            tracing::error!("Failed to update worker status: {:?}", err);
                        };
                    }
                },
                {
                    let repo = repo.clone();
                    async move {
                        // 工作者失败时的逻辑
                        if let Err(err) = repo
                            .update_worker_status(&workflow_name, &worker_name, WorkerStatus::Failed)
                            .await
                        {
                            tracing::error!("Failed to update worker status: {:?}", err);
                        };
                    }
                },
            )
            .await?;
        Ok(())
    }

    pub async fn workflow_exists(&self, name: &str) -> Result<bool> {
        self.repo.workflow_exists(name).await
    }
    pub async fn worker_exists(&self, workflow_name: &str, worker_name: &str) -> Result<bool> {
        self.repo.worker_exists(workflow_name, worker_name).await
    }
    pub async fn get_workflow(&self, name: &str) -> Result<Option<Workflow>> {
        if let Some(row) = self.repo.get_workflow(name).await? {
            let mut workflow: Workflow = row.into();
            let worker_entities: Vec<WorkerEntity> = self.repo.get_workers_by_workflow(name).await?;
            let workers: Vec<Worker> = worker_entities
                .into_iter()
                .map(|entity| entity.into())
                .collect();
            workflow.workers = workers;
            return Ok(Some(workflow));
        }
        Ok(None)
    }
    pub async fn list_workflows(&self) -> Result<Vec<WorkflowEntity>> {
        self.repo.get_all_workflows().await
    }

    /// 获取分页工作流列表
    pub async fn list_workflows_paged(
        &self,
        page: u64,
        page_size: u64,
        status_filter: Option<String>,
        sort_by: String,
        order: String,
    ) -> Result<(Vec<WorkflowEntity>, u64)> {
        self.repo.get_workflows_paged(
            page,
            page_size,
            status_filter.as_deref(),
            &sort_by,
            &order,
        ).await
    }

    pub async fn get_workflow_info(&self, name: &str) -> Result<Option<WorkflowEntity>> {
        self.repo.get_workflow(name).await
    }

    pub async fn list_workers(&self, workflow_name: &str) -> Result<Vec<WorkerEntity>> {
        self.repo.get_workers_by_workflow(workflow_name).await
    }

    pub async fn create_workflow(&self, workflow: Workflow) -> Result<()> {
        tracing::info!("Creating workflow: {:?}", workflow);
        
        // 判断 workflow name 是否存在
        if self.workflow_exists(&workflow.name).await? {
            tracing::warn!("Workflow already exists: {:?}", workflow);
            return Err(eyre::eyre!("Workflow already exists"));
        }
        
        // 验证依赖关系
        if let Err(e) = workflow.validate_dependencies() {
            tracing::error!("Workflow dependency validation failed: {:?}", e);
            return Err(e);
        }
        
        self.repo.insert_workflow(&workflow).await?;
        for worker in &workflow.workers {
            if self.worker_exists(&worker.workflow_name, &worker.name).await? {
                tracing::warn!("Worker already exists: {:?}", worker);
                return Err(eyre::eyre!("Worker already exists"));
            }
        }
        self.repo.insert_workers(&workflow.workers).await?;
        tracing::info!("Workflow created successfully: {:?}", workflow);
        Ok(())
    }

    pub async fn run_workflow(&self, workflow_name: &String) -> Result<()> {
        let workflow = self
            .get_workflow(workflow_name)
            .await?
            .ok_or_else(|| eyre::eyre!("Workflow not found"))?;
        
        tracing::info!("Running workflow: {:?}", workflow);
        
        // 获取拓扑排序后的执行顺序
        let execution_levels = workflow.get_execution_order()
            .map_err(|e| eyre::eyre!("Failed to get execution order: {}", e))?;
        
        // 创建 worker 名称到 worker 对象的映射，避免在循环中克隆名称
        let worker_map: HashMap<String, Worker> = workflow.workers
            .into_iter()
            .map(|w| {
                let name = w.name.clone();
                (name, w)
            })
            .collect();
        
        // 按层级顺序执行，同一层级内并发执行
        for (level_idx, level_workers) in execution_levels.into_iter().enumerate() {
            tracing::info!("Starting execution level {}: {:?}", level_idx, level_workers);
            
            // 为当前层级的所有 worker 创建任务
            let mut level_tasks = Vec::new();
            
            for worker_name in &level_workers {
                if let Some(worker) = worker_map.get(worker_name) {
                    let coordinator = self.clone();
                    let worker = worker.clone();
                    let worker_name = worker_name.clone(); // 只在需要时克隆
                    
                    let task = tokio::spawn(async move {
                        tracing::info!("Starting worker: {}", worker.name);
                        if let Err(e) = coordinator.run_workers(worker).await {
                            tracing::error!("Error occurred while running worker {}: {:?}", worker_name, e);
                            return Err(e);
                        }
                        tracing::info!("Worker {} completed", worker_name);
                        Ok(())
                    });
                    
                    level_tasks.push(task);
                }
            }
            
            // 等待当前层级的所有 worker 完成
            for task in level_tasks {
                match task.await {
                    Ok(Ok(())) => {
                        // Worker 成功完成
                    }
                    Ok(Err(worker_err)) => {
                        tracing::error!("Worker error: {:?}", worker_err);
                        return Err(worker_err);
                    }
                    Err(join_err) => {
                        tracing::error!("Task join error: {:?}", join_err);
                        return Err(eyre::eyre!("Task join error: {}", join_err));
                    }
                }
            }
            
            tracing::info!("Execution level {} completed", level_idx);
        }
        
        tracing::info!("Workflow {} completed successfully", workflow_name);
        Ok(())
    }
    pub async fn list_topic(&self, name: Option<String>) -> eyre::Result<Vec<String>> {
        let selector = match name {
            Some(name) => format!("@/*/router/subscriber/wattle/{}/**", name),
            None => "@/*/router/subscriber/wattle/**".to_string(),
        };
        let replies = self.session.get(&selector).await.map_err(|e| eyre!(e))?;
        let mut topics = Vec::new();
        while let Ok(reply) = replies.recv_async().await {
            let sample = reply
                .result()
                .map_err(|e| eyre!("Failed to get result from reply: {}", e))?;
            let key = sample.key_expr().to_string();
            let splites = key.split("/").collect::<Vec<_>>()[6..].join("/");
            topics.push(splites);
        }
        Ok(topics)
    }

    // 兼容性方法，提供服务访问
    pub fn service(&self) -> &Repositories {
        &self.repo
    }

    // 工作者管理方法
    pub async fn get_worker_info(&self, workflow_name: &str, worker_name: &str) -> Result<WorkerInfo> {
        let worker = self.repo.get_worker(workflow_name, worker_name).await?
            .ok_or_else(|| eyre::eyre!("Worker not found"))?;
        
        Ok(WorkerInfo {
            name: worker.name,
            workflow_name: worker.workflow_name,
            command: worker.command,
            args: worker.args,
            working_dir: worker.working_dir,
            env_vars: worker.env_vars,
            status: "running".to_string(), // 从状态获取
        })
    }

    pub async fn start_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        let worker = self.repo.get_worker(workflow_name, worker_name).await?
            .ok_or_else(|| eyre::eyre!("Worker not found"))?;
        self.run_workers(worker).await
    }

    pub async fn stop_worker(&self, _workflow_name: &str, _worker_name: &str) -> Result<()> {
        // 这里应该实现停止工作者的逻辑
        // 可以通过PID管理来终止进程
        Ok(())
    }

    pub async fn delete_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        self.repo.delete_worker(workflow_name, worker_name).await
    }

    pub async fn get_worker_logs(&self, workflow_name: &str, worker_name: &str) -> Result<Vec<String>> {
        let logs = self.repo.get_worker_logs(workflow_name, worker_name).await?;
        Ok(logs.into_iter().map(|log| format!("{}: {}", log.log_type, log.file_path)).collect())
    }
}
