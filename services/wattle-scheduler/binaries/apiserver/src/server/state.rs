use core::Settings;
use coordinator::Coordinator;
use std::sync::Arc;

#[allow(unused)]
#[derive(Clone)]
pub struct AppState {
    pub config: Settings,
    pub coordinator: Arc<Coordinator>,
}

impl AppState {
    pub async fn new(config: Settings) -> eyre::Result<Self> {
        // 使用 from_settings 方法直接创建协调器
        let coordinator = Arc::new(Coordinator::from_settings(config.clone()).await?);

        let pid_maps = coordinator.get_pid_maps();
        tracing::info!("pid_maps: {:?}", pid_maps);

        Ok(AppState { config, coordinator })
    }
}
