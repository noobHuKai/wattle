use crate::server::{
    models::*,
    response::{ApiJsonResult, IntoApiJsonResult},
    AppState,
};

use axum::{
    extract::{Path, State},
    response::sse::{Event, Sse},
    routing::{delete, get, post},
    Json, Router,
};
use futures::Stream;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashMap;
use std::convert::Infallible;
use std::time::Duration;
use wattle_scheduler::{TaskGroupSummary, TaskInfo, TaskSummary};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskSchemaResponse {
    pub input: Option<HashMap<String, Value>>,
    pub output: Option<HashMap<String, Value>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskMetricsResponse {
    pub metrics: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerMetricsResponse {
    pub metrics: Vec<String>,
}

pub fn routes() -> Router<AppState> {
    Router::new()
        .route("/", get(list_all_tasks))
        .route("/", post(create_task))
        .route("/{task_group_name}", get(list_tasks))
        .route("/{task_group_name}/{task_name}", get(get_task_info))
        .route("/{task_group_name}/{task_name}", delete(delete_task))
        .route("/{task_group_name}/{task_name}/log", get(get_task_log))
        .route(
            "/{task_group_name}/{task_name}/schema",
            get(get_task_schema),
        )
        .route(
            "/{task_group_name}/{task_name}/metrics",
            get(get_task_metrics),
        )
        .route("/server/metrics", get(get_server_metrics))
        .route(
            "/{task_group_name}/{task_name}/log/sse",
            get(get_task_log_sse),
        )
        .route("/{task_group_name}/{task_name}/stop", post(stop_task))
        .route("/{task_group_name}/{task_name}/start", post(start_task))
}

/// 列出指定任务组中的所有任务
async fn list_tasks(
    State(state): State<AppState>,
    Path(task_group_name): Path<String>,
) -> ApiJsonResult<Vec<TaskSummary>> {
    tracing::info!("获取任务组 {} 的任务列表", task_group_name);

    let tasks = state
        .scheduler
        .list_tasks(&task_group_name)
        .await
        .map_err(|e| {
            tracing::error!("获取任务组 {} 的任务列表失败: {}", task_group_name, e);
            e
        })?;

    tasks.into_success_json()
}

/// 列出所有任务
async fn list_all_tasks(
    State(state): State<AppState>,
) -> ApiJsonResult<Vec<(TaskGroupSummary, Vec<TaskSummary>)>> {
    tracing::info!("获取所有任务列表");

    let all_tasks = state.scheduler.list_all_tasks().await.map_err(|e| {
        tracing::error!("获取所有任务列表失败: {}", e);
        e
    })?;

    all_tasks.into_success_json()
}

/// 获取任务详细信息
async fn get_task_info(
    State(state): State<AppState>,
    Path((task_group_name, task_name)): Path<(String, String)>,
) -> ApiJsonResult<TaskInfo> {
    tracing::info!("获取任务信息: {}/{}", task_group_name, task_name);

    let task_info = state
        .scheduler
        .get_task_info(&task_group_name, &task_name)
        .await
        .map_err(|e| {
            tracing::error!("获取任务 {}/{} 信息失败: {}", task_group_name, task_name, e);
            e
        })?;

    task_info.into_success_json()
}

/// 获取任务的JSON Schema
async fn get_task_schema(
    State(state): State<AppState>,
    Path((task_group_name, task_name)): Path<(String, String)>,
) -> ApiJsonResult<TaskSchemaResponse> {
    tracing::info!("获取任务Schema: {}/{}", task_group_name, task_name);

    // 从数据库获取任务的schema信息
    let task_model = state
        .scheduler
        .service()
        .get_task(&task_group_name, &task_name)
        .await
        .map_err(|e| {
            tracing::error!(
                "获取任务 {}/{} Schema失败: {}",
                task_group_name,
                task_name,
                e
            );
            e
        })?;

    let schema_response = TaskSchemaResponse {
        input: task_model.get_input_schema(),
        output: task_model.get_output_schema(),
    };

    schema_response.into_success_json()
}

/// 获取任务的metrics
async fn get_task_metrics(
    State(state): State<AppState>,
    Path((task_group_name, task_name)): Path<(String, String)>,
) -> ApiJsonResult<TaskMetricsResponse> {
    tracing::info!("获取任务Metrics: {}/{}", task_group_name, task_name);

    let metrics = state
        .scheduler
        .service()
        .get_task_metrics(&task_group_name, &task_name)
        .await
        .map_err(|e| {
            tracing::error!(
                "获取任务 {}/{} Metrics失败: {}",
                task_group_name,
                task_name,
                e
            );
            e
        })?;

    let metrics_response = TaskMetricsResponse { metrics };

    metrics_response.into_success_json()
}

/// 获取服务器的metrics
async fn get_server_metrics(State(state): State<AppState>) -> ApiJsonResult<ServerMetricsResponse> {
    tracing::info!("获取服务器Metrics");

    let metrics = state
        .scheduler
        .service()
        .get_server_metrics()
        .await
        .map_err(|e| {
            tracing::error!("获取服务器Metrics失败: {}", e);
            e
        })?;

    let metrics_response = ServerMetricsResponse { metrics };

    metrics_response.into_success_json()
}

/// 创建任务
async fn create_task(
    State(state): State<AppState>,
    Json(req): Json<CreateTaskRequest>,
) -> ApiJsonResult<String> {
    tracing::info!(
        "在任务组 {} 中创建任务: {}",
        req.task_group_name,
        req.spec.name.as_ref().unwrap_or(&"default".to_string())
    );

    state
        .scheduler
        .add_task(&req.task_group_name, req.spec.clone())
        .await
        .map_err(|e| {
            tracing::error!(
                "在任务组 {} 中创建任务 {} 失败: {}",
                req.task_group_name,
                req.spec.name.as_ref().unwrap_or(&"default".to_string()),
                e
            );
            e
        })?;

    format!(
        "任务 {}/{} 创建成功",
        req.task_group_name,
        req.spec.name.as_ref().unwrap_or(&"default".to_string())
    )
    .into_success_json()
}

/// 删除任务
async fn delete_task(
    State(state): State<AppState>,
    Path((task_group_name, task_name)): Path<(String, String)>,
) -> ApiJsonResult<String> {
    tracing::info!("删除任务: {}/{}", task_group_name, task_name);

    state
        .scheduler
        .delete_task(&task_group_name, &task_name)
        .await
        .map_err(|e| {
            tracing::error!("删除任务 {}/{} 失败: {}", task_group_name, task_name, e);
            e
        })?;

    format!("任务 {}/{} 删除成功", task_group_name, task_name).into_success_json()
}

/// 获取任务日志
async fn get_task_log(
    State(state): State<AppState>,
    Path((task_group_name, task_name)): Path<(String, String)>,
) -> ApiJsonResult<TaskLog> {
    tracing::info!("获取任务日志: {}/{}", task_group_name, task_name);

    let log_results = state
        .scheduler
        .get_task_log(&task_group_name, &task_name)
        .await
        .map_err(|e| {
            tracing::error!("获取任务 {}/{} 日志失败: {}", task_group_name, task_name, e);
            e
        })?;

    // 处理日志结果，将日志文件分类为stdout和stderr
    let mut stdout_content = String::new();
    let mut stderr_content = String::new();

    for log in log_results {
        match log.log_type.as_str() {
            "stdout" => {
                if let Ok(content) = tokio::fs::read_to_string(&log.file_path).await {
                    stdout_content.push_str(&content);
                }
            }
            "stderr" => {
                if let Ok(content) = tokio::fs::read_to_string(&log.file_path).await {
                    stderr_content.push_str(&content);
                }
            }
            _ => {}
        }
    }

    let task_log = TaskLog {
        stdout: if stdout_content.is_empty() {
            None
        } else {
            Some(stdout_content)
        },
        stderr: if stderr_content.is_empty() {
            None
        } else {
            Some(stderr_content)
        },
        timestamp: chrono::Utc::now().to_rfc3339(),
    };

    task_log.into_success_json()
}

/// 使用 SSE 实时获取任务日志
async fn get_task_log_sse(
    State(state): State<AppState>,
    Path((task_group_name, task_name)): Path<(String, String)>,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    tracing::info!("开始 SSE 日志流: {}/{}", task_group_name, task_name);

    let scheduler = state.scheduler.clone();
    let tg_name = task_group_name.clone();
    let t_name = task_name.clone();

    let stream = async_stream::stream! {
        // 首先获取历史日志
        if let Ok(log_results) = scheduler.get_task_log(&tg_name, &t_name).await {
            for log in log_results {
                if let Ok(content) = tokio::fs::read_to_string(&log.file_path).await {
                    let log_entry = LogEntry {
                        timestamp: log.created_at.to_rfc3339(),
                        level: log.log_type,
                        content,
                    };

                    let event = Event::default()
                        .data(serde_json::to_string(&log_entry).unwrap_or_default());
                    yield Ok(event);
                }
            }
        }

        // 发送完成事件
        let end_event = Event::default()
            .event("end")
            .data("Log stream ended");
        yield Ok(end_event);
    };

    Sse::new(stream).keep_alive(
        axum::response::sse::KeepAlive::new()
            .interval(Duration::from_secs(1))
            .text("keep-alive-text"),
    )
}

/// 停止任务
async fn stop_task(
    State(state): State<AppState>,
    Path((task_group_name, task_name)): Path<(String, String)>,
) -> ApiJsonResult<String> {
    tracing::info!("停止任务: {}/{}", task_group_name, task_name);

    state
        .scheduler
        .stop_task(&task_group_name, &task_name)
        .await
        .map_err(|e| {
            tracing::error!("停止任务 {}/{} 失败: {}", task_group_name, task_name, e);
            e
        })?;

    format!("任务 {}/{} 停止成功", task_group_name, task_name).into_success_json()
}

/// 启动任务
async fn start_task(
    State(state): State<AppState>,
    Path((task_group_name, task_name)): Path<(String, String)>,
) -> ApiJsonResult<String> {
    tracing::info!("启动任务: {}/{}", task_group_name, task_name);

    state
        .scheduler
        .start_task(&task_group_name, &task_name)
        .await
        .map_err(|e| {
            tracing::error!("启动任务 {}/{} 失败: {}", task_group_name, task_name, e);
            e
        })?;

    format!("任务 {}/{} 启动成功", task_group_name, task_name).into_success_json()
}
