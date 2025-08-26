use core::{SchedulerConfig, Task, TaskGroup, TaskStatus};
use std::{collections::HashMap, process::Command, sync::Arc};

use eyre::{Result, eyre};
use rand::Rng as _;
use runtime::TaskExecutor;
use storage::{Repositories, TaskEntity, TaskGroupEntity};
use tokio::{net::TcpListener, sync::RwLock};
use tracing::{error, info};

#[derive(Clone)]
pub struct Scheduler {
    repo: Arc<Repositories>,
    executor: TaskExecutor,
    session: zenoh::Session,
}

impl Scheduler {
    pub async fn new(cfg: SchedulerConfig) -> Result<Self> {
        let repo = Arc::new(Repositories::new(cfg.db_url).await?);
        let executor = TaskExecutor::new(cfg.execution);

        let session = zenoh::open(zenoh::Config::default())
            .await
            .map_err(|e| eyre!(e))?;

        if let Err(e) = Self::run_zenohd().await {
            error!("Failed to start zenohd: {}", e);
        }

        Ok(Scheduler {
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

    async fn run_tasks(&self, task: Task) -> Result<()> {
        let task_group_name = task.group_name.clone();
        let task_name = task.name.clone();
        self.executor
            .execute(
                task,
                async || {
                    // 任务创建时的逻辑
                    if let Err(err) = self
                        .repo
                        .update_task_status(&task_group_name, &task_name, TaskStatus::Running)
                        .await
                    {
                        tracing::error!("Failed to update task status: {:?}", err);
                    };
                },
                async || {
                    // 任务成功时的逻辑
                    if let Err(err) = self
                        .repo
                        .update_task_status(&task_group_name, &task_name, TaskStatus::Completed)
                        .await
                    {
                        tracing::error!("Failed to update task status: {:?}", err);
                    };
                },
                async || {
                    // 任务失败时的逻辑
                    if let Err(err) = self
                        .repo
                        .update_task_status(&task_group_name, &task_name, TaskStatus::Failed)
                        .await
                    {
                        tracing::error!("Failed to update task status: {:?}", err);
                    };
                },
            )
            .await?;
        Ok(())
    }

    pub async fn task_group_exists(&self, name: &str) -> Result<bool> {
        self.repo.task_group_exists(name).await
    }
    pub async fn task_exists(&self, group_name: &str, task_name: &str) -> Result<bool> {
        self.repo.task_exists(group_name, task_name).await
    }
    pub async fn get_task_group(&self, name: &str) -> Result<Option<TaskGroup>> {
        if let Some(row) = self.repo.get_task_group(name).await? {
            let mut task_group: TaskGroup = row.into();
            let task_entities: Vec<TaskEntity> = self.repo.get_tasks_by_group(name).await?;
            let tasks: Vec<Task> = task_entities
                .into_iter()
                .map(|entity| entity.into())
                .collect();
            task_group.tasks = tasks;
            return Ok(Some(task_group));
        }
        Ok(None)
    }
    pub async fn list_task_groups(&self) -> Result<Vec<TaskGroupEntity>> {
        self.repo.get_all_task_groups().await
    }

    pub async fn get_task_group_info(&self, name: &str) -> Result<Option<TaskGroupEntity>> {
        self.repo.get_task_group(name).await
    }

    pub async fn list_tasks(&self, group_name: &str) -> Result<Vec<TaskEntity>> {
        self.repo.get_tasks_by_group(group_name).await
    }

    pub async fn create_task_group(&self, task_group: TaskGroup) -> Result<()> {
        tracing::info!("Creating task group: {:?}", task_group);
        // 判断 task_group name 是否存在
        if self.task_group_exists(&task_group.name).await? {
            tracing::warn!("Task group already exists: {:?}", task_group);
            return Err(eyre::eyre!("Task group already exists"));
        }
        self.repo.insert_task_group(&task_group).await?;
        for task in &task_group.tasks {
            if self.task_exists(&task.group_name, &task.name).await? {
                tracing::warn!("Task already exists: {:?}", task);
                return Err(eyre::eyre!("Task already exists"));
            }
        }
        self.repo.insert_tasks(&task_group.tasks).await?;
        tracing::info!("Task group created successfully: {:?}", task_group);
        Ok(())
    }

    pub async fn run_task_group(&self, task_group_name: &String) -> Result<()> {
        let task_group = self
            .get_task_group(task_group_name)
            .await?
            .ok_or_else(|| eyre::eyre!("Task group not found"))?;
        tracing::info!("Running task group: {:?}", task_group);
        for task in task_group.tasks {
            let scheduler = self.clone();
            tokio::spawn(async move {
                if let Err(e) = scheduler.run_tasks(task).await {
                    tracing::error!("Error occurred while running task: {:?}", e);
                }
            });
        }
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
}
