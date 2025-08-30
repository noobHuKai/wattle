pub mod worker;
pub mod workflow;

pub use super::AppState;
use std::time::Duration;

use axum::Router;
use tower_http::{timeout::TimeoutLayer, trace::TraceLayer};

pub async fn api_routes(state: AppState) -> eyre::Result<Router> {
    Ok(Router::new()
        .nest(
            "/api",
            Router::new()
                .nest("/workflows", workflow::routes())
                .nest("/workers", worker::routes()),
        )
        .layer((
            // Tracing
            TraceLayer::new_for_http(),
            // Timeout
            TimeoutLayer::new(Duration::from_secs(30)),
        ))
        .with_state(state))
}
