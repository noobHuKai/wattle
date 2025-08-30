use std::path::PathBuf;

use crate::server::{
    AppState,
    models::{ListQuery, PagedResponse, PaginationMeta, WorkflowResponse},
    response::{ApiJsonResult, IntoApiJsonResult},
};

use axum::{
    Json, Router,
    extract::{Path, Query, State},
    routing::{get, post},
};
use coordinator::{Worker, WorkerEntity, Workflow, WorkflowEntity, WorkflowSpec};

pub fn routes() -> Router<AppState> {
    Router::new()
        .route("/", get(list_workflows))
        .route("/", post(create_workflow))
        .route("/{workflow_name}", get(get_workflow_info))
        .route("/{workflow_name}/start", post(start_workflow))
        .route("/{workflow_name}/workers", get(get_workers_by_workflow))
}

/// 列出所有工作流
async fn list_workflows(
    State(state): State<AppState>,
    Query(query): Query<ListQuery>,
) -> ApiJsonResult<PagedResponse<WorkflowResponse>> {
    let ListQuery { page, page_size, status, sort_by, order } = query;
    
    let page = page.unwrap_or(1).max(1);
    let page_size = page_size.unwrap_or(20).min(100).max(1); // 限制最大100条
    let sort_by = sort_by.unwrap_or_else(|| "created_at".to_string());
    let order = order.unwrap_or_else(|| "desc".to_string());
    
    let (workflows, total) = state
        .coordinator
        .list_workflows_paged(page, page_size, status, Some(sort_by), Some(order))
        .await?;

    let workflow_responses: Vec<WorkflowResponse> = workflows
        .into_iter()
        .map(WorkflowResponse::from)
        .collect();

    let total_pages = (total + page_size - 1) / page_size;
    let pagination = PaginationMeta {
        page,
        page_size,
        total,
        total_pages,
        has_next: page < total_pages,
        has_prev: page > 1,
    };

    let response = PagedResponse {
        data: workflow_responses,
        pagination,
    };

    response.into_success_json()
}

/// 获取工作流详细信息
async fn get_workflow_info(
    State(state): State<AppState>,
    Path(workflow_name): Path<String>,
) -> ApiJsonResult<WorkflowEntity> {
    let workflow_info = state
        .coordinator
        .get_workflow_info(&workflow_name)
        .await?;
    let workflow_info = workflow_info.ok_or_else(|| eyre::eyre!("工作流 {} 不存在", workflow_name))?;
    workflow_info.into_success_json()
}

/// 创建工作流
async fn create_workflow(
    State(state): State<AppState>,
    Json(mut req): Json<WorkflowSpec>,
) -> ApiJsonResult<String> {
    let name = req.name.clone().unwrap_or_else(|| "default".to_string());

    let mut inc_name = name.clone();
    let mut idx = 1;
    while state.coordinator.workflow_exists(&inc_name).await? {
        inc_name = format!("{name}-{idx}");
        idx += 1;
    }
    req.name = Some(inc_name.clone());
    let workflow_name = inc_name;

    let current_dir = PathBuf::from(req.current_dir.clone().unwrap_or_else(|| ".".to_string()));

    let mut workers: Vec<Worker> = vec![];
    for (worker_name, mut worker_spec) in req.services {
        let worker_name = worker_spec
            .name
            .unwrap_or_else(|| format!("{}-{}", workflow_name, worker_name));

        let mut worker_inc_name = worker_name.clone();
        let mut idx = 1;
        while state
            .coordinator
            .worker_exists(&workflow_name, &worker_inc_name)
            .await?
        {
            worker_inc_name = format!("{workflow_name}-{worker_name}-{idx}");
            idx += 1;
        }
        worker_spec.name = Some(worker_inc_name);
        if let Some(working_dir) = &worker_spec.working_dir {
            let working_dir = PathBuf::from(working_dir);
            if working_dir.is_relative() {
                worker_spec.working_dir = Some(current_dir.join(working_dir).display().to_string());
            }
        } else {
            worker_spec.working_dir = Some(current_dir.clone().display().to_string());
        }

        let mut worker: Worker = worker_spec.into();
        worker.workflow_name = workflow_name.clone();
        workers.push(worker);
    }
    let workflow = Workflow {
        name: workflow_name.clone(),
        working_dir: Some(current_dir.display().to_string()),
        workers,
    };

    state.coordinator.create_workflow(workflow).await?;
    Ok(workflow_name).into_success_json()
}

/// 启动工作流
async fn start_workflow(
    State(state): State<AppState>,
    Path(workflow_name): Path<String>,
) -> ApiJsonResult<String> {
    state.coordinator.run_workflow(&workflow_name).await?;

    workflow_name.into_success_json()
}

async fn get_workers_by_workflow(
    State(state): State<AppState>,
    Path(workflow_name): Path<String>,
) -> ApiJsonResult<Vec<WorkerEntity>> {
    let workers = state.coordinator.list_workers(&workflow_name).await?;
    workers.into_success_json()
}
