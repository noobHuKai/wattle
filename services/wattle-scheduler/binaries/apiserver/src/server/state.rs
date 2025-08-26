use crate::Config;
use scheduler::Scheduler;
use std::sync::Arc;

#[allow(unused)]
#[derive(Clone)]
pub struct AppState {
    pub config: Config,
    pub scheduler: Arc<Scheduler>,
}

impl AppState {
    pub async fn new(config: Config) -> eyre::Result<Self> {
        // 构建执行器配置
        let executor_config = config.scheduler.clone().unwrap_or_default();

        // 初始化调度器
        let scheduler = Arc::new(Scheduler::new(executor_config).await?);

        let pid_maps = scheduler.get_pid_maps();
        super::metrics::collect_metrics(pid_maps).await?;

        Ok(AppState { config, scheduler })
    }
}
