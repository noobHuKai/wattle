use std::{collections::HashMap, sync::Arc, time::Duration};

use prometheus::{Gauge, GaugeVec, register_gauge, register_gauge_vec};
use sysinfo::{Pid, ProcessesToUpdate, System};
use tokio::{sync::RwLock, time::interval};

/// 单个进程的指标
#[derive(Debug, Clone)]
pub struct ProcMetrics {
    pub cpu: f64,
    pub memory: u64, // MB
}

/// 指标收集器
pub struct MetricsCollector {
    sys: System,
    // Prometheus 指标
    cpu_usage: Gauge,
    mem_usage: Gauge,
    scheduler_cpu_usage: Gauge,
    scheduler_mem_usage: Gauge,

    proc_cpu: GaugeVec,
    proc_mem: GaugeVec,
}

impl MetricsCollector {
    pub fn new() -> eyre::Result<Self> {
        // 注册 Prometheus 指标

        let cpu_usage = register_gauge!("system_cpu_usage_percent", "系统 CPU 使用率")?;
        let mem_usage = register_gauge!("system_memory_usage_percent", "系统内存使用率")?;

        let scheduler_cpu_usage =
            register_gauge!("scheduler_cpu_usage_percent", "调度器 CPU 使用率")?;
        let scheduler_mem_usage =
            register_gauge!("scheduler_memory_usage_percent", "调度器内存使用率")?;

        let proc_cpu = register_gauge_vec!(
            "task_cpu_usage_percent",
            "任务 CPU 使用率",
            &["group_name", "task_name", "pid"]
        )?;
        let proc_mem = register_gauge_vec!(
            "task_memory_usage_mb",
            "任务内存使用量（MB）",
            &["group_name", "task_name", "pid"]
        )?;

        Ok(MetricsCollector {
            sys: System::new_all(),
            cpu_usage,
            mem_usage,
            scheduler_cpu_usage,
            scheduler_mem_usage,
            proc_cpu,
            proc_mem,
        })
    }

    /// 刷新系统信息
    fn refresh_system(&mut self) {
        self.sys.refresh_cpu_all();
        self.sys.refresh_memory();
        self.sys.refresh_processes(ProcessesToUpdate::All, true);
    }

    /// 收集系统整体指标
    pub fn collect_system_metrics(&mut self) -> eyre::Result<()> {
        self.refresh_system();

        // 计算平均 CPU 使用率
        let cpu_usage = if !self.sys.cpus().is_empty() {
            self.sys
                .cpus()
                .iter()
                .map(|cpu| cpu.cpu_usage())
                .sum::<f32>()
                / self.sys.cpus().len() as f32
        } else {
            0.0
        } as f64;
        self.cpu_usage.set(cpu_usage);

        // 计算内存使用率
        let total_memory = self.sys.total_memory();
        let used_memory = self.sys.used_memory();
        let memory_usage_percent = if total_memory > 0 {
            (used_memory as f64 / total_memory as f64) * 100.0
        } else {
            0.0
        };
        self.mem_usage.set(memory_usage_percent);

        Ok(())
    }

    /// 收集当前进程指标
    pub fn collect_self_metrics(&mut self) -> eyre::Result<Option<ProcMetrics>> {
        let self_pid = std::process::id();
        let self_pid_sys = Pid::from_u32(self_pid);

        if let Some(proc) = self.sys.process(self_pid_sys) {
            let metrics = ProcMetrics {
                cpu: proc.cpu_usage() as f64,
                memory: proc.memory() / 1024 / 1024, // 转换为 MB
            };

            // 更新 Prometheus 指标 - 主进程
            self.scheduler_cpu_usage.set(metrics.cpu);
            self.scheduler_mem_usage.set(metrics.memory as f64);

            Ok(Some(metrics))
        } else {
            Ok(None)
        }
    }

    /// 收集子进程指标
    pub async fn collect_children_metrics(
        &mut self,
        pid_map: Arc<RwLock<HashMap<u32, (String, String)>>>,
    ) -> eyre::Result<Vec<ProcMetrics>> {
        let self_pid = std::process::id();
        let self_pid_sys = Pid::from_u32(self_pid);
        let mut result = Vec::new();

        // 获取 PID 映射的只读锁
        let pid_mapping = pid_map.read().await;

        if let Some(parent_proc) = self.sys.process(self_pid_sys) {
            let parent_pid = parent_proc.pid();

            for (pid, proc) in self.sys.processes() {
                if proc.parent() == Some(parent_pid) {
                    let pid_u32 = pid.as_u32();
                    let metrics = ProcMetrics {
                        cpu: proc.cpu_usage() as f64,
                        memory: proc.memory() / 1024 / 1024, // 转换为 MB
                    };

                    // 从 PID 映射获取任务信息
                    let (group_name, task_name) = pid_mapping
                        .get(&pid_u32)
                        .cloned()
                        .unwrap_or_else(|| ("unknown".to_string(), "unknown".to_string()));

                    // 更新 Prometheus 指标 - 子进程
                    self.proc_cpu
                        .with_label_values(&[&group_name, &task_name, &pid_u32.to_string()])
                        .set(metrics.cpu);
                    self.proc_mem
                        .with_label_values(&[&group_name, &task_name, &pid_u32.to_string()])
                        .set(metrics.memory as f64);

                    result.push(metrics);
                }
            }
        }

        Ok(result)
    }
}

pub async fn collect_metrics(
    pid_map: Arc<RwLock<HashMap<u32, (String, String)>>>,
) -> eyre::Result<()> {
    let mut collector = MetricsCollector::new()?;

    tokio::spawn(async move {
        let mut interval = interval(Duration::from_secs(5));
        loop {
            interval.tick().await;
            // 收集系统整体指标
            if let Err(err) = collector.collect_system_metrics() {
                tracing::error!("收集系统指标失败: {}", err);
            };

            // 收集主进程指标
            if let Err(err) = collector.collect_self_metrics() {
                tracing::error!("主进程指标收集失败: {}", err);
            };

            // 收集子进程指标
            if let Err(err) = collector.collect_children_metrics(pid_map.clone()).await {
                tracing::error!("收集子进程指标失败: {}", err);
            };
        }
    });

    Ok(())
}
