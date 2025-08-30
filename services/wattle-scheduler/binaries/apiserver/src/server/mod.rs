use core::Settings;
use tokio::signal;

mod apis;
mod metrics;
mod models;
mod response;
mod state;

pub use state::AppState;

/// 运行服务器
pub async fn run_server(cfg: Settings) -> eyre::Result<()> {
    let host = cfg.server.host.as_ref().unwrap_or(&"localhost".to_string()).clone();
    let port = cfg.server.port.unwrap_or(9240);
    let addr = format!("{}:{}", host, port);
    let listener = tokio::net::TcpListener::bind(&addr).await?;

    let shared_state = AppState::new(cfg).await?;

    let app = apis::api_routes(shared_state).await?;

    tracing::info!("Listen {}...", addr);
    axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal())
        .await?;

    Ok(())
}

/// 优雅关闭服务器
async fn shutdown_signal() {
    let ctrl_c = async {
        signal::ctrl_c()
            .await
            .expect("failed to install Ctrl+C handler");
    };

    #[cfg(unix)]
    let terminate = async {
        signal::unix::signal(signal::unix::SignalKind::terminate())
            .expect("failed to install signal handler")
            .recv()
            .await;
    };

    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();

    tokio::select! {
        _ = ctrl_c => {},
        _ = terminate => {},
    }
}
