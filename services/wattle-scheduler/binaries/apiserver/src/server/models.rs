use serde::{Deserialize, Serialize};
use storage::{WorkflowEntity, WorkerEntity};

/// API 查询参数
#[derive(Debug, Deserialize)]
pub struct ListQuery {
    pub page: Option<u64>,
    pub page_size: Option<u64>,
    pub status: Option<String>,
    pub sort_by: Option<String>,
    pub order: Option<String>, // asc, desc
}

impl Default for ListQuery {
    fn default() -> Self {
        Self {
            page: Some(1),
            page_size: Some(20),
            status: None,
            sort_by: Some("created_at".to_string()),
            order: Some("desc".to_string()),
        }
    }
}

/// 分页响应包装器
#[derive(Debug, Serialize)]
pub struct PagedResponse<T> {
    pub data: Vec<T>,
    pub pagination: PaginationMeta,
}

#[derive(Debug, Serialize)]
pub struct PaginationMeta {
    pub page: u64,
    pub page_size: u64,
    pub total: u64,
    pub total_pages: u64,
    pub has_next: bool,
    pub has_prev: bool,
}

/// API 专用的工作流响应模型
#[derive(Debug, Serialize)]
pub struct WorkflowResponse {
    pub name: String,
    pub working_dir: Option<String>,
    pub status: String,
    pub created_at: String,
    pub started_at: Option<String>,
    pub completed_at: Option<String>,
}

impl From<WorkflowEntity> for WorkflowResponse {
    fn from(entity: WorkflowEntity) -> Self {
        Self {
            name: entity.name,
            working_dir: entity.working_dir,
            status: entity.status,
            created_at: entity.created_at,
            started_at: entity.started_at,
            completed_at: entity.completed_at,
        }
    }
}

/// API 专用的工作者响应模型
#[derive(Debug, Serialize)]
pub struct WorkerResponse {
    pub name: String,
    pub workflow_name: String,
    pub command: String,
    pub status: String,
    pub created_at: String,
    pub started_at: Option<String>,
    pub completed_at: Option<String>,
}

impl From<WorkerEntity> for WorkerResponse {
    fn from(entity: WorkerEntity) -> Self {
        Self {
            name: entity.name,
            workflow_name: entity.workflow_name,
            command: entity.command,
            status: entity.status,
            created_at: entity.created_at,
            started_at: entity.started_at,
            completed_at: entity.completed_at,
        }
    }
}
