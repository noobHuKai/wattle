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
        // 构建执行器配置
        let executor_config = config.coordinator.clone().unwrap_or_default();

        // 初始化协调器
        let coordinator = Arc::new(Coordinator::new(executor_config).await?);

        let pid_maps = coordinator.get_pid_maps();
        tracing::info!("pid_maps: {:?}", pid_maps);

        Ok(AppState { config, coordinator })
    }
}
