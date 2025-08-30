use core::{ExecutionConfig, Worker};
use eyre::{eyre, Result};
use std::{collections::HashMap, future::Future, path::PathBuf, process::Stdio, sync::Arc};
use tokio::{
    fs,
    io::BufReader,
    process::{Child, Command},
    sync::RwLock,
};
use tracing::{debug, error, info};

#[derive(Clone)]
pub struct TaskExecutor {
    config: ExecutionConfig,
    pub pid_maps: Arc<RwLock<HashMap<u32, (String, String)>>>,
}

impl TaskExecutor {
    pub fn new(config: ExecutionConfig) -> Self {
        Self {
            config,
            pid_maps: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    async fn get_log_paths(base_log_dir: &PathBuf, worker: &Worker) -> Result<(PathBuf, PathBuf)> {
        let worker_log_dir = base_log_dir
            .join(&worker.workflow_name)
            .join(&worker.name);

        // 确保日志目录存在
        if !worker_log_dir.exists() {
            fs::create_dir_all(&worker_log_dir)
                .await
                .map_err(|e| eyre::eyre!("创建日志目录失败: {}", e))?;
            debug!("创建日志目录: {:?}", worker_log_dir);
        }

        let stdout_path = worker_log_dir.join("stdout.log");
        let stderr_path = worker_log_dir.join("stderr.log");

        Ok((stdout_path, stderr_path))
    }

    fn parse_cmd(worker: &Worker) -> Result<(String, Vec<String>)> {
        let cmd_with_args = &worker.command;
        let command_parts: Vec<String> = shell_words::split(cmd_with_args)?;
        if command_parts.is_empty() {
            return Err(eyre!("命令不能为空"));
        }

        let (cmd, mut cmd_args) = if command_parts.len() > 1 {
            (command_parts[0].clone(), command_parts[1..].to_vec())
        } else {
            (command_parts[0].clone(), vec![])
        };
        if let Some(args) = &worker.args {
            cmd_args.extend_from_slice(args);
        }

        // 特殊处理 - 添加 Zenoh 连接参数
        if cmd.contains("zenoh") || cmd_args.iter().any(|arg| arg.contains("zenoh")) {
            cmd_args.extend_from_slice(&[
                "--connect".to_string(),
                "tcp/127.0.0.1:7447".to_string(),
            ]);
        }

        // 为某些程序添加适当的参数
        if cmd.contains("java") {
            cmd_args.insert(0, "-Xms256m".to_string());
            cmd_args.insert(1, "-Xmx1g".to_string());
        } else if cmd.contains("python") {
            cmd_args.insert(0, "-u".to_string()); // 无缓冲输出
        }

        debug!("解析命令: {} {:?}", cmd, cmd_args);
        Ok((cmd, cmd_args))
    }

    async fn setup_log_streaming(
        &self,
        mut child: Child,
        stdout_path: PathBuf,
        stderr_path: PathBuf,
    ) -> Result<Child> {
        // 获取 stdout 和 stderr
        let stdout = child.stdout.take().ok_or_else(|| eyre!("无法获取 stdout"))?;
        let stderr = child.stderr.take().ok_or_else(|| eyre!("无法获取 stderr"))?;

        // 启动 stdout 流处理任务
        let stdout_task = tokio::spawn(Self::stream_to_file(stdout, stdout_path));
        let stderr_task = tokio::spawn(Self::stream_to_file(stderr, stderr_path));

        // 在后台等待流任务完成
        tokio::spawn(async move {
            let _ = tokio::try_join!(stdout_task, stderr_task);
        });

        Ok(child)
    }

    async fn stream_to_file(
        stream: impl tokio::io::AsyncRead + Unpin,
        file_path: PathBuf,
    ) -> Result<()> {
        let file = fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&file_path)
            .await?;

        let mut reader = BufReader::new(stream);
        let mut writer = tokio::io::BufWriter::new(file);
        tokio::io::copy_buf(&mut reader, &mut writer).await?;
        tokio::io::AsyncWriteExt::flush(&mut writer).await?;

        Ok(())
    }

    fn setup_environment(worker: &Worker) -> Result<HashMap<String, String>> {
        let mut env_vars = std::env::vars().collect::<HashMap<String, String>>();

        // 添加工作者特定的环境变量
        env_vars.insert("WORKER_NAME".to_string(), worker.name.clone());
        env_vars.insert("WORKFLOW_NAME".to_string(), worker.workflow_name.clone());

        // 添加用户定义的环境变量
        if let Some(user_env) = &worker.env_vars {
            for (key, value) in user_env {
                env_vars.insert(key.clone(), value.clone());
            }
        }

        // 添加 Zenoh 相关环境变量
        env_vars.insert("ZENOH_ROUTER".to_string(), "tcp/127.0.0.1:7447".to_string());
        env_vars.insert("ZENOH_MODE".to_string(), "client".to_string());

        Ok(env_vars)
    }

    async fn start_process(
        &self,
        worker: &Worker,
        (cmd, args): (String, Vec<String>),
        (stdout_path, stderr_path): (PathBuf, PathBuf),
        env_vars: HashMap<String, String>,
    ) -> Result<Child> {
        let working_dir = worker
            .working_dir
            .as_ref()
            .map(PathBuf::from)
            .unwrap_or_else(|| std::env::current_dir().unwrap());

        info!(
            "启动进程: {} {:?} in {:?}",
            cmd, args, working_dir
        );

        let mut command = Command::new(&cmd);
        command
            .args(&args)
            .current_dir(&working_dir)
            .envs(&env_vars)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .stdin(Stdio::null())
            .kill_on_drop(true);

        let child = command.spawn().map_err(|e| {
            error!("启动进程失败: {}", e);
            eyre!("启动进程失败: {}", e)
        })?;

        info!("进程启动成功，PID: {}", child.id().unwrap_or(0));

        // 设置日志流
        let child_with_logs = self
            .setup_log_streaming(child, stdout_path, stderr_path)
            .await?;

        Ok(child_with_logs)
    }

    pub async fn execute<OnCreate, OnSuccess, OnFail>(
        &self,
        worker: Worker,
        on_create: OnCreate,
        on_success: OnSuccess,
        on_fail: OnFail,
    ) -> Result<()>
    where
        OnCreate: Future<Output = ()> + Send,
        OnSuccess: Future<Output = ()> + Send,
        OnFail: Future<Output = ()> + Send,
    {
        // 解析命令
        let (cmd, args) = Self::parse_cmd(&worker)?;

        // 获取日志路径
        let default_log_dir = PathBuf::from("logs");
        let log_dir = if let Some(ref log_dir_str) = self.config.log_dir {
            PathBuf::from(log_dir_str)
        } else {
            default_log_dir
        };
        let (stdout_path, stderr_path) = Self::get_log_paths(&log_dir, &worker).await?;

        // 设置环境变量
        let env_vars = Self::setup_environment(&worker)?;

        // 调用创建回调
        on_create.await;

        // 启动进程
        let mut child = self
            .start_process(&worker, (cmd, args), (stdout_path, stderr_path), env_vars)
            .await?;

        // 记录 PID 映射
        if let Some(pid) = child.id() {
            let mut pid_maps = self.pid_maps.write().await;
            pid_maps.insert(pid, (worker.workflow_name.clone(), worker.name.clone()));
        }

        // 等待进程完成
        let exit_status = child.wait().await.map_err(|e| {
            error!("等待进程完成时发生错误: {}", e);
            eyre!("等待进程完成时发生错误: {}", e)
        })?;

        // 清理 PID 映射
        if let Some(pid) = child.id() {
            let mut pid_maps = self.pid_maps.write().await;
            pid_maps.remove(&pid);
        }

        info!("进程已完成，退出状态: {:?}", exit_status);

        // 根据退出状态调用相应的回调
        if exit_status.success() {
            info!("工作者 {} 成功完成", worker.name);
            on_success.await;
        } else {
            error!("工作者 {} 执行失败，退出码: {:?}", worker.name, exit_status.code());
            on_fail.await;
        }

        Ok(())
    }

    pub async fn list_running_processes(&self) -> Vec<(u32, String, String)> {
        let pid_maps = self.pid_maps.read().await;
        pid_maps
            .iter()
            .map(|(pid, (workflow_name, worker_name))| (*pid, workflow_name.clone(), worker_name.clone()))
            .collect()
    }

    pub async fn kill_process(&self, pid: u32) -> Result<()> {
        #[cfg(unix)]
        {
            use std::process::Command;
            let output = Command::new("kill").arg(pid.to_string()).output()?;
            if !output.status.success() {
                return Err(eyre!("杀死进程失败: {}", String::from_utf8_lossy(&output.stderr)));
            }
        }

        #[cfg(windows)]
        {
            use std::process::Command;
            let output = Command::new("taskkill").args(&["/F", "/PID", &pid.to_string()]).output()?;
            if !output.status.success() {
                return Err(eyre!("杀死进程失败: {}", String::from_utf8_lossy(&output.stderr)));
            }
        }

        // 清理 PID 映射
        let mut pid_maps = self.pid_maps.write().await;
        pid_maps.remove(&pid);

        info!("进程 {} 已被杀死", pid);
        Ok(())
    }

    pub async fn kill_workflow_processes(&self, workflow_name: &str) -> Result<Vec<u32>> {
        let pid_maps = self.pid_maps.read().await;
        let pids_to_kill: Vec<u32> = pid_maps
            .iter()
            .filter_map(|(pid, (wf_name, _))| {
                if wf_name == workflow_name {
                    Some(*pid)
                } else {
                    None
                }
            })
            .collect();

        drop(pid_maps); // 释放读锁

        let mut killed_pids = Vec::new();
        for pid in pids_to_kill {
            if let Err(e) = self.kill_process(pid).await {
                error!("杀死进程 {} 失败: {}", pid, e);
            } else {
                killed_pids.push(pid);
            }
        }

        Ok(killed_pids)
    }
}
