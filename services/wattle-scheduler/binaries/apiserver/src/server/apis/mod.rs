// pub mod task;
pub mod task_group;

pub use super::AppState;
use std::time::Duration;

use axum::Router;
use tower_http::{timeout::TimeoutLayer, trace::TraceLayer};

pub async fn api_routes(state: AppState) -> eyre::Result<Router> {
    Ok(Router::new()
        .nest(
            "/api",
            Router::new().nest("/task_groups", task_group::routes()), // .nest("/tasks", task::routes()),
        )
        .layer((
            // Tracing
            TraceLayer::new_for_http(),
            // Timeout
            TimeoutLayer::new(Duration::from_secs(30)),
        ))
        .with_state(state))
}
