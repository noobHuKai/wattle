use std::path::PathBuf;

use crate::server::{
    AppState,
    response::{ApiJsonResult, IntoApiJsonResult},
};

use axum::{
    Json, Router,
    extract::{Path, State},
    routing::{get, post},
};
use scheduler::{Task, TaskEntity, TaskGroup, TaskGroupEntity, TaskGroupSpec};

pub fn routes() -> Router<AppState> {
    Router::new()
        .route("/", get(list_task_groups))
        .route("/", post(create_task_group))
        .route("/{task_group_name}", get(get_task_group_info))
        .route("/{task_group_name}/start", post(start_task_group))
        .route("/{task_group_name}/tasks", get(get_tasks_by_group))
}

/// 列出所有任务组
async fn list_task_groups(State(state): State<AppState>) -> ApiJsonResult<Vec<TaskGroupEntity>> {
    let groups = state.scheduler.list_task_groups().await?;

    groups.into_success_json()
}

/// 获取任务组详细信息
async fn get_task_group_info(
    State(state): State<AppState>,
    Path(task_group_name): Path<String>,
) -> ApiJsonResult<TaskGroupEntity> {
    let group_info = state
        .scheduler
        .get_task_group_info(&task_group_name)
        .await?;
    let group_info = group_info.ok_or_else(|| eyre::eyre!("任务组 {} 不存在", task_group_name))?;
    group_info.into_success_json()
}

/// 创建任务组
async fn create_task_group(
    State(state): State<AppState>,
    Json(mut req): Json<TaskGroupSpec>,
) -> ApiJsonResult<String> {
    let name = req.name.clone().unwrap_or_else(|| "default".to_string());

    let mut inc_name = name.clone();
    let mut idx = 1;
    while state.scheduler.task_group_exists(&inc_name).await? {
        inc_name = format!("{name}-{idx}");
        idx += 1;
    }
    req.name = Some(inc_name.clone());
    let task_group_name = inc_name;

    let current_dir = PathBuf::from(req.current_dir.clone().unwrap_or_else(|| ".".to_string()));

    let mut tasks: Vec<Task> = vec![];
    for (task_name, mut task_spec) in req.services {
        let task_name = task_spec
            .name
            .unwrap_or_else(|| format!("{}-{}", task_group_name, task_name));

        let mut task_inc_name = task_name.clone();
        let mut idx = 1;
        while state
            .scheduler
            .task_exists(&task_group_name, &task_inc_name)
            .await?
        {
            task_inc_name = format!("{task_group_name}-{task_name}-{idx}");
            idx += 1;
        }
        task_spec.name = Some(task_inc_name);
        if let Some(working_dir) = &task_spec.working_dir {
            let working_dir = PathBuf::from(working_dir);
            if working_dir.is_relative() {
                task_spec.working_dir = Some(current_dir.join(working_dir).display().to_string());
            }
        } else {
            task_spec.working_dir = Some(current_dir.clone().display().to_string());
        }

        let mut task: Task = task_spec.into();
        task.group_name = task_group_name.clone();
        tasks.push(task);
    }
    let task_group = TaskGroup {
        name: task_group_name.clone(),
        working_dir: Some(current_dir.display().to_string()),
        tasks,
    };

    state.scheduler.create_task_group(task_group).await?;
    Ok(task_group_name).into_success_json()
}

/// 启动任务组
async fn start_task_group(
    State(state): State<AppState>,
    Path(task_group_name): Path<String>,
) -> ApiJsonResult<String> {
    state.scheduler.run_task_group(&task_group_name).await?;

    task_group_name.into_success_json()
}

async fn get_tasks_by_group(
    State(state): State<AppState>,
    Path(task_group_name): Path<String>,
) -> ApiJsonResult<Vec<TaskEntity>> {
    let tasks = state.scheduler.list_tasks(&task_group_name).await?;
    tasks.into_success_json()
}
