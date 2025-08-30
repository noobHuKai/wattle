use crate::server::{
    response::{ApiJsonResult, IntoApiJsonResult},
    AppState,
};

use axum::{
    extract::{Path, State},
    response::sse::{Event, Sse},
    routing::{delete, get, post},
    Router,
};
use futures::Stream;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashMap;
use std::convert::Infallible;
use std::time::Duration;
use coordinator::{WorkerInfo, WorkerSummary};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkerSchemaResponse {
    pub input: Option<HashMap<String, Value>>,
    pub output: Option<HashMap<String, Value>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkerMetricsResponse {
    pub metrics: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerMetricsResponse {
    pub metrics: Vec<String>,
}

pub fn routes() -> Router<AppState> {
    Router::new()
        .route("/{workflow_name}", get(list_workers))
        .route("/{workflow_name}/{worker_name}", get(get_worker_info))
        .route("/{workflow_name}/{worker_name}/schema", get(get_worker_schema))
        .route("/{workflow_name}/{worker_name}/start", post(start_worker))
        .route("/{workflow_name}/{worker_name}/stop", post(stop_worker))
        .route("/{workflow_name}/{worker_name}/restart", post(restart_worker))
        .route("/{workflow_name}/{worker_name}/logs", get(get_worker_logs))
        .route("/{workflow_name}/{worker_name}/logs/stream", get(stream_worker_logs))
        .route("/{workflow_name}/{worker_name}/metrics", get(get_worker_metrics))
        .route("/{workflow_name}/{worker_name}", delete(delete_worker))
}

/// 列出指定工作流中的所有工作者
async fn list_workers(
    State(state): State<AppState>,
    Path(workflow_name): Path<String>,
) -> ApiJsonResult<Vec<WorkerSummary>> {
    tracing::info!("列出工作流 {} 的所有工作者", workflow_name);

    let workers = state
        .coordinator
        .list_workers(&workflow_name)
        .await
        .map_err(|e| {
            tracing::error!("获取工作流 {} 工作者列表失败: {}", workflow_name, e);
            e
        })?;

    let worker_summaries: Vec<WorkerSummary> = workers
        .into_iter()
        .map(|entity| WorkerSummary {
            name: entity.name,
            workflow_name: entity.workflow_name,
            status: entity.status,
            command: entity.command,
            created_at: entity.created_at,
            started_at: entity.started_at,
            completed_at: entity.completed_at,
        })
        .collect();

    worker_summaries.into_success_json()
}

/// 获取工作者详细信息
async fn get_worker_info(
    State(state): State<AppState>,
    Path((workflow_name, worker_name)): Path<(String, String)>,
) -> ApiJsonResult<WorkerInfo> {
    tracing::info!("获取工作者信息: {}/{}", workflow_name, worker_name);

    let worker_info = state
        .coordinator
        .get_worker_info(&workflow_name, &worker_name)
        .await
        .map_err(|e| {
            tracing::error!("获取工作者 {}/{} 信息失败: {}", workflow_name, worker_name, e);
            e
        })?;

    worker_info.into_success_json()
}

/// 获取工作者的JSON Schema
async fn get_worker_schema(
    State(state): State<AppState>,
    Path((workflow_name, worker_name)): Path<(String, String)>,
) -> ApiJsonResult<WorkerSchemaResponse> {
    tracing::info!("获取工作者Schema: {}/{}", workflow_name, worker_name);

    // 从数据库获取工作者的schema信息
    let _worker_model = state
        .coordinator
        .service()
        .get_worker(&workflow_name, &worker_name)
        .await
        .map_err(|e| {
            tracing::error!(
                "获取工作者 {}/{} Schema失败: {}",
                workflow_name,
                worker_name,
                e
            );
            e
        })?
        .ok_or_else(|| eyre::eyre!("Worker not found"))?;

    let schema_response = WorkerSchemaResponse {
        input: None,  // 暂时返回None，可以后续实现
        output: None, // 暂时返回None，可以后续实现
    };

    schema_response.into_success_json()
}

/// 启动工作者
async fn start_worker(
    State(state): State<AppState>,
    Path((workflow_name, worker_name)): Path<(String, String)>,
) -> ApiJsonResult<String> {
    tracing::info!("启动工作者: {}/{}", workflow_name, worker_name);

    state
        .coordinator
        .start_worker(&workflow_name, &worker_name)
        .await
        .map_err(|e| {
            tracing::error!("启动工作者 {}/{} 失败: {}", workflow_name, worker_name, e);
            e
        })?;

    format!("Worker {}/{} started", workflow_name, worker_name).into_success_json()
}

/// 停止工作者
async fn stop_worker(
    State(state): State<AppState>,
    Path((workflow_name, worker_name)): Path<(String, String)>,
) -> ApiJsonResult<String> {
    tracing::info!("停止工作者: {}/{}", workflow_name, worker_name);

    state
        .coordinator
        .stop_worker(&workflow_name, &worker_name)
        .await
        .map_err(|e| {
            tracing::error!("停止工作者 {}/{} 失败: {}", workflow_name, worker_name, e);
            e
        })?;

    format!("Worker {}/{} stopped", workflow_name, worker_name).into_success_json()
}

/// 重启工作者
async fn restart_worker(
    State(state): State<AppState>,
    Path((workflow_name, worker_name)): Path<(String, String)>,
) -> ApiJsonResult<String> {
    tracing::info!("重启工作者: {}/{}", workflow_name, worker_name);

    // 先停止
    let _ = state
        .coordinator
        .stop_worker(&workflow_name, &worker_name)
        .await;

    // 等待一点时间
    tokio::time::sleep(Duration::from_millis(1000)).await;

    // 再启动
    state
        .coordinator
        .start_worker(&workflow_name, &worker_name)
        .await
        .map_err(|e| {
            tracing::error!("重启工作者 {}/{} 失败: {}", workflow_name, worker_name, e);
            e
        })?;

    format!("Worker {}/{} restarted", workflow_name, worker_name).into_success_json()
}

/// 获取工作者日志
async fn get_worker_logs(
    State(state): State<AppState>,
    Path((workflow_name, worker_name)): Path<(String, String)>,
) -> ApiJsonResult<Vec<String>> {
    tracing::info!("获取工作者日志: {}/{}", workflow_name, worker_name);

    let logs = state
        .coordinator
        .get_worker_logs(&workflow_name, &worker_name)
        .await
        .map_err(|e| {
            tracing::error!("获取工作者 {}/{} 日志失败: {}", workflow_name, worker_name, e);
            e
        })?;

    logs.into_success_json()
}

/// 流式获取工作者日志
async fn stream_worker_logs(
    State(state): State<AppState>,
    Path((workflow_name, worker_name)): Path<(String, String)>,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    tracing::info!("流式获取工作者日志: {}/{}", workflow_name, worker_name);

    let stream = async_stream::stream! {
        let mut interval = tokio::time::interval(Duration::from_secs(1));
        loop {
            interval.tick().await;
            
            match state.coordinator.get_worker_logs(&workflow_name, &worker_name).await {
                Ok(logs) => {
                    for log in logs {
                        let event = Event::default().data(log);
                        yield Ok(event);
                    }
                }
                Err(e) => {
                    let error_event = Event::default().data(format!("Error: {}", e));
                    yield Ok(error_event);
                    break;
                }
            }
        }
    };

    Sse::new(stream).keep_alive(
        axum::response::sse::KeepAlive::new()
            .interval(Duration::from_secs(10))
            .text("keep-alive-text"),
    )
}

/// 获取工作者指标
async fn get_worker_metrics(
    State(_state): State<AppState>,
    Path((workflow_name, worker_name)): Path<(String, String)>,
) -> ApiJsonResult<WorkerMetricsResponse> {
    tracing::info!("获取工作者指标: {}/{}", workflow_name, worker_name);

    // 这里应该从监控系统获取指标数据
    let metrics = vec![
        format!("worker_status: running"),
        format!("worker_uptime: 3600s"),
        format!("cpu_usage: 15%"),
        format!("memory_usage: 256MB"),
    ];

    WorkerMetricsResponse { metrics }.into_success_json()
}

/// 删除工作者
async fn delete_worker(
    State(state): State<AppState>,
    Path((workflow_name, worker_name)): Path<(String, String)>,
) -> ApiJsonResult<String> {
    tracing::info!("删除工作者: {}/{}", workflow_name, worker_name);

    state
        .coordinator
        .delete_worker(&workflow_name, &worker_name)
        .await
        .map_err(|e| {
            tracing::error!("删除工作者 {}/{} 失败: {}", workflow_name, worker_name, e);
            e
        })?;

    format!("Worker {}/{} deleted", workflow_name, worker_name).into_success_json()
}
