use core::{ExecutionConfig, Task, TaskStatus};
use eyre::Result;
use std::{collections::HashMap, path::PathBuf, process::Stdio, sync::Arc};
use tokio::{fs, process::Command, sync::RwLock};
use tracing::{debug, error, info, warn};

#[derive(Debug, Clone)]
pub struct TaskExecutor {
    config: ExecutionConfig,
    pub pid_maps: Arc<RwLock<HashMap<u32, (String, String)>>>, // pid -> (group, task)
}

impl TaskExecutor {
    pub fn new(config: Option<ExecutionConfig>) -> Self {
        TaskExecutor {
            config: config.unwrap_or_default(),
            pid_maps: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    async fn get_log_paths(base_log_dir: &PathBuf, task: &Task) -> Result<(PathBuf, PathBuf)> {
        let log_dir = base_log_dir.join(&task.group_name).join(&task.name);

        // 确保日志目录存在
        if !log_dir.exists() {
            fs::create_dir_all(&log_dir)
                .await
                .map_err(|e| eyre::eyre!("创建日志目录失败: {}", e))?;
            debug!("创建日志目录: {:?}", log_dir);
        }

        let stdout_path = log_dir.join("stdout.log");
        let stderr_path = log_dir.join("stderr.log");

        Ok((stdout_path, stderr_path))
    }

    fn parse_cmd(task: &Task) -> Result<(String, Vec<String>)> {
        let command_parts = task
            .command
            .trim()
            .split_whitespace()
            .map(|x| x.to_string())
            .collect::<Vec<String>>();
        if command_parts.is_empty() {
            return Err(eyre::eyre!("命令不能为空"));
        }

        let (cmd, mut cmd_args) = if command_parts.len() > 1 {
            (command_parts[0].clone(), command_parts[1..].to_vec())
        } else {
            (command_parts[0].clone(), vec![])
        };
        if let Some(args) = &task.args {
            cmd_args.extend(args.clone());
        }
        Ok((cmd, cmd_args))
    }
    fn detect_virtual_environment(working_dir: &String) -> Option<PathBuf> {
        let working_dir = PathBuf::from(working_dir);
        // 常见的虚拟环境目录名称
        let venv_dirs = ["venv", ".venv", "env", ".env"];

        for venv_dir in &venv_dirs {
            let venv_path = working_dir.join(venv_dir);
            if !venv_path.exists() {
                continue;
            }

            // 检查不同操作系统下的Python可执行文件路径
            let python_paths = if cfg!(windows) {
                vec![
                    venv_path.join("Scripts").join("python.exe"),
                    venv_path.join("Scripts").join("python3.exe"),
                ]
            } else {
                vec![
                    venv_path.join("bin").join("python"),
                    venv_path.join("bin").join("python3"),
                ]
            };

            for python_path in python_paths {
                if python_path.exists() {
                    debug!("Detected Python: {:?}", python_path);
                    return Some(python_path);
                }
            }
        }

        None
    }
    fn setup_environment(task: &Task) -> Result<HashMap<String, String>> {
        // 继承系统环境变量（可选，根据安全需求决定）
        let mut env = std::env::vars().collect::<HashMap<String, String>>();

        // 添加任务规格中定义的环境变量
        if let Some(env_vars) = &task.env_vars {
            for (key, value) in env_vars {
                env.insert(key.clone(), value.clone());
                debug!("set env vars: {}={}", key, value);
            }
        }

        // 添加Wattle内置环境变量
        env.insert("WATTLE_TASK_NAME".to_string(), task.name.clone());
        env.insert("WATTLE_GROUP_NAME".to_string(), task.group_name.clone());

        let cmd = task
            .command
            .trim()
            .split_whitespace()
            .next()
            .unwrap_or("")
            .to_string();

        // 检测虚拟环境并设置相关环境变量
        if let Some(working_dir) = &task.working_dir {
            if cmd == "python" || cmd == "uv" || cmd == "python3" {
                if let Some(venv_python) = Self::detect_virtual_environment(working_dir) {
                    if let Some(venv_root) = venv_python.parent().and_then(|p| p.parent()) {
                        env.insert(
                            "VIRTUAL_ENV".to_string(),
                            venv_root.to_string_lossy().to_string(),
                        );

                        // 更新PATH以包含虚拟环境的bin目录
                        let venv_bin = if cfg!(windows) {
                            venv_root.join("Scripts")
                        } else {
                            venv_root.join("bin")
                        };

                        if let Some(current_path) = env.get("PATH") {
                            let new_path =
                                format!("{}:{}", venv_bin.to_string_lossy(), current_path);
                            env.insert("PATH".to_string(), new_path);
                        } else {
                            env.insert("PATH".to_string(), venv_bin.to_string_lossy().to_string());
                        }

                        debug!(
                            "Set Virtual Env: VIRTUAL_ENV={}",
                            venv_root.to_string_lossy()
                        );
                    }
                }
                // 刷新缓冲区
                env.insert("PYTHONUNBUFFERED".to_string(), "1".to_string());
            }
        }

        Ok(env)
    }
    pub async fn execute<OnCreate, OnSuccess, OnFailure>(
        &self,
        task: Task,
        on_create: OnCreate,
        on_success: OnSuccess,
        on_failure: OnFailure,
    ) -> Result<()>
    where
        OnCreate: AsyncFnOnce(),
        OnSuccess: AsyncFnOnce(),
        OnFailure: AsyncFnOnce(),
    {
        tracing::info!("Starting execution of task: {:?}", task);
        // 1. 准备日志路径
        let (stdout_path, stderr_path) =
            Self::get_log_paths(&self.config.log_dir.clone().unwrap_or_default(), &task).await?;

        let command_parts = task
            .command
            .trim()
            .split_whitespace()
            .map(|x| x.to_string())
            .collect::<Vec<String>>();

        if command_parts.is_empty() {
            return Err(eyre::eyre!("命令不能为空"));
        }

        // 2. 准备命令和参数
        let (cmd, args) = Self::parse_cmd(&task)?;
        // 3. 构建命令
        let mut command = Command::new(cmd);
        if !args.is_empty() {
            command.args(args);
        }

        // 4. 准备工作目录
        if let Some(working_dir) = &task.working_dir {
            command.current_dir(working_dir);
        }

        // 5. 准备环境变量
        let envs = Self::setup_environment(&task)?;
        command.envs(envs);

        // 6.  重定向输出到文件
        let stdout_file = fs::File::create(&stdout_path).await?;
        let stderr_file = fs::File::create(&stderr_path).await?;
        command.stdout(Stdio::from(stdout_file.into_std().await));
        command.stderr(Stdio::from(stderr_file.into_std().await));

        let start_time = std::time::Instant::now();

        // 调用创建回调
        on_create().await;

        let mut child = command.spawn()?;
        let pid_opt = child.id(); // Option<u32>
        if let Some(pid) = pid_opt {
            // 将 pid -> (group, task) 写入 map，便于外部监控/管理
            let mut map = self.pid_maps.write().await;
            map.insert(pid, (task.group_name.clone(), task.name.clone()));
        }

        // 等待结果，若超时则尝试杀死子进程
        let child_result = if let Some(timeout) = self.config.timeout.or(self.config.timeout) {
            match tokio::time::timeout(timeout, child.wait()).await {
                Ok(wait_res) => Ok(wait_res),
                Err(elapsed) => {
                    // 超时：尝试终止子进程
                    warn!("任务 {} 超时，尝试终止 PID {:?}", task.name, pid_opt);
                    if let Err(e) = child.kill().await {
                        warn!("尝试终止进程失败: {:?}", e);
                    }
                    // 尝试等待子进程结束以避免僵尸进程
                    let _ = child.wait().await;
                    Err(elapsed)
                }
            }
        } else {
            Ok(child.wait().await)
        };
        let end_time = std::time::Instant::now();
        let elapsed = end_time - start_time;
        // 8. 处理执行结果
        match child_result {
            Ok(Ok(exit_status)) => {
                let exit_code = exit_status.code();
                let status = if exit_status.success() {
                    TaskStatus::Completed
                } else {
                    TaskStatus::Failed
                };

                info!(
                    "任务 {} 执行完成，退出码: {:?}，状态: {:?}，耗时: {}ms",
                    task.name,
                    exit_code,
                    status,
                    elapsed.as_millis()
                );
                if status == TaskStatus::Completed {
                    on_success().await;
                } else {
                    on_failure().await;
                }
                Ok(())
            }
            Ok(Err(e)) => {
                error!("任务 {} 进程执行错误: {}", task.name, e);
                on_failure().await;
                Ok(())
            }
            Err(_) => {
                warn!("任务 {} 执行超时", task.name);
                on_failure().await;
                Ok(())
            }
        }
    }
}
