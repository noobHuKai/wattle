use crate::Config;
use coordinator::Coordinator;
use std::sync::Arc;

#[allow(unused)]
#[derive(Clone)]
pub struct AppState {
    pub config: Config,
    pub coordinator: Arc<Coordinator>,
}

impl AppState {
    pub async fn new(config: Config) -> eyre::Result<Self> {
        // 构建协调器和执行器配置
        let coordinator_config = config.coordinator.clone().unwrap_or_default();
        let executor_config = core::ExecutionConfig::default();

        // 初始化协调器
        let coordinator = Arc::new(Coordinator::new(coordinator_config, executor_config).await?);

        let pid_maps = coordinator.get_pid_maps();
        tracing::info!("pid_maps: {:?}", pid_maps);

        Ok(AppState { config, coordinator })
    }
}
