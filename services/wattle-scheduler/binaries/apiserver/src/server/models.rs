use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
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

/// 工作流响应 DTO
#[derive(Debug, Serialize)]
pub struct WorkflowResponse {
    pub name: String,
    pub status: String,
    pub created_at: DateTime<Utc>,
    pub started_at: Option<DateTime<Utc>>,
    pub completed_at: Option<DateTime<Utc>>,
    pub workers_count: usize,
    pub completed_workers: usize,
    pub failed_workers: usize,
}

/// 工作流详情响应 DTO
#[derive(Debug, Serialize)]
pub struct WorkflowDetailResponse {
    pub name: String,
    pub status: String,
    pub created_at: DateTime<Utc>,
    pub started_at: Option<DateTime<Utc>>,
    pub completed_at: Option<DateTime<Utc>>,
    pub workers: Vec<WorkerSummaryResponse>,
    pub execution_time: Option<f64>, // 执行时间（秒）
}

/// 工作者摘要响应 DTO（用于列表和工作流详情）
#[derive(Debug, Serialize)]
pub struct WorkerSummaryResponse {
    pub name: String,
    pub status: String,
    pub created_at: DateTime<Utc>,
    pub started_at: Option<DateTime<Utc>>,
    pub completed_at: Option<DateTime<Utc>>,
    pub execution_time: Option<f64>, // 执行时间（秒）
}

/// 工作者详情响应 DTO
#[derive(Debug, Serialize)]
pub struct WorkerDetailResponse {
    pub name: String,
    pub workflow_name: String,
    pub command: String,
    pub args: Option<Vec<String>>,
    pub working_dir: Option<String>,
    pub env_vars: Option<std::collections::HashMap<String, String>>,
    pub status: String,
    pub created_at: DateTime<Utc>,
    pub started_at: Option<DateTime<Utc>>,
    pub completed_at: Option<DateTime<Utc>>,
    pub execution_time: Option<f64>,
    pub exit_code: Option<i32>,
    pub error_message: Option<String>,
}

/// 工作者响应 DTO（保持向后兼容）
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

/// 系统状态响应 DTO
#[derive(Debug, Serialize)]
pub struct SystemStatusResponse {
    pub active_workflows: usize,
    pub running_workers: usize,
    pub completed_workflows: usize,
    pub failed_workflows: usize,
    pub uptime: f64, // 系统运行时间（秒）
    pub memory_usage: Option<u64>, // 内存使用量（字节）
    pub cpu_usage: Option<f64>, // CPU使用率（百分比）
}

/// 日志条目响应 DTO
#[derive(Debug, Serialize)]
pub struct LogEntryResponse {
    pub timestamp: DateTime<Utc>,
    pub level: String,
    pub message: String,
    pub worker_name: String,
    pub workflow_name: String,
}

// 实现转换逻辑
impl WorkflowResponse {
    // Note: from_entity_with_stats function removed as it was unused
}

impl From<WorkflowEntity> for WorkflowResponse {
    fn from(entity: WorkflowEntity) -> Self {
        Self {
            name: entity.name,
            status: entity.status,
            created_at: parse_datetime(&entity.created_at),
            started_at: entity.started_at.as_ref().map(|s| parse_datetime(s)),
            completed_at: entity.completed_at.as_ref().map(|s| parse_datetime(s)),
            workers_count: 0,
            completed_workers: 0,
            failed_workers: 0,
        }
    }
}

impl From<WorkerEntity> for WorkerSummaryResponse {
    fn from(entity: WorkerEntity) -> Self {
        let execution_time = if let (Some(started), Some(completed)) = (&entity.started_at, &entity.completed_at) {
            let start = parse_datetime(started);
            let end = parse_datetime(completed);
            Some((end - start).num_milliseconds() as f64 / 1000.0)
        } else {
            None
        };

        Self {
            name: entity.name,
            status: entity.status,
            created_at: parse_datetime(&entity.created_at),
            started_at: entity.started_at.as_ref().map(|s| parse_datetime(s)),
            completed_at: entity.completed_at.as_ref().map(|s| parse_datetime(s)),
            execution_time,
        }
    }
}

impl From<WorkerEntity> for WorkerDetailResponse {
    fn from(entity: WorkerEntity) -> Self {
        let execution_time = if let (Some(started), Some(completed)) = (&entity.started_at, &entity.completed_at) {
            let start = parse_datetime(started);
            let end = parse_datetime(completed);
            Some((end - start).num_milliseconds() as f64 / 1000.0)
        } else {
            None
        };

        // 解析 JSON 字符串字段
        let args = entity.args.and_then(|s| serde_json::from_str(&s).ok());
        let env_vars = entity.env_vars.and_then(|s| serde_json::from_str(&s).ok());

        Self {
            name: entity.name,
            workflow_name: entity.workflow_name,
            command: entity.command,
            args,
            working_dir: entity.working_dir,
            env_vars,
            status: entity.status,
            created_at: parse_datetime(&entity.created_at),
            started_at: entity.started_at.as_ref().map(|s| parse_datetime(s)),
            completed_at: entity.completed_at.as_ref().map(|s| parse_datetime(s)),
            execution_time,
            exit_code: None, // WorkerEntity does not have exit_code field
            error_message: entity.error_message,
        }
    }
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

// 辅助函数
fn parse_datetime(datetime_str: &str) -> DateTime<Utc> {
    DateTime::parse_from_rfc3339(datetime_str)
        .unwrap_or_else(|_| {
            // 如果解析失败，尝试其他格式或返回当前时间
            chrono::Utc::now().into()
        })
        .with_timezone(&Utc)
}


