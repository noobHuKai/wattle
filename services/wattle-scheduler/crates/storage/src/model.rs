use core::{Task, TaskGroup};

use serde::{Deserialize, Serialize};
use sqlx::FromRow;

/// 数据库中的任务组实体
#[derive(Debug, Clone, Serialize, Deserialize, FromRow)]
pub struct TaskGroupEntity {
    pub name: String,
    pub working_dir: Option<String>,
    pub status: String,
    pub created_at: String, // SQLite 存储为 TEXT
    pub deleted_at: Option<String>,
    pub started_at: Option<String>,
    pub completed_at: Option<String>,
}

impl Into<TaskGroup> for TaskGroupEntity {
    fn into(self) -> TaskGroup {
        TaskGroup {
            name: self.name,
            working_dir: self.working_dir,
            tasks: Vec::new(), // 需要单独查询
        }
    }
}

/// 数据库中的任务实体
#[derive(Debug, Clone, Serialize, Deserialize, FromRow)]
pub struct TaskEntity {
    pub group_name: String,
    pub name: String,
    pub command: String,
    pub args: Option<String>, // JSON 字符串
    pub working_dir: Option<String>,
    pub env_vars: Option<String>, // JSON 字符串
    pub status: String,
    pub error_message: Option<String>,
    pub created_at: String,
    pub deleted_at: Option<String>,
    pub started_at: Option<String>,
    pub completed_at: Option<String>,
}
impl Into<Task> for TaskEntity {
    fn into(self) -> Task {
        Task {
            group_name: self.group_name,
            name: self.name,
            command: self.command,
            args: if let Some(args_str) = self.args {
                serde_json::from_str(&args_str).ok()
            } else {
                None
            },
            working_dir: self.working_dir,
            env_vars: if let Some(env_str) = self.env_vars {
                serde_json::from_str(&env_str).ok()
            } else {
                None
            },
        }
    }
}

/// 数据库中的任务日志实体
#[derive(Debug, Clone, Serialize, Deserialize, FromRow)]
pub struct TaskLogEntity {
    pub id: i64,
    pub group_name: String,
    pub task_name: String,
    pub log_type: String,
    pub file_path: String,
    pub created_at: String,
}

/// 任务实体转换为业务模型的方法
impl TaskEntity {
    pub fn to_task(&self) -> Result<core::Task, serde_json::Error> {
        let args = if let Some(args_str) = &self.args {
            Some(serde_json::from_str(args_str)?)
        } else {
            None
        };

        let env_vars = if let Some(env_str) = &self.env_vars {
            Some(serde_json::from_str(env_str)?)
        } else {
            None
        };

        Ok(core::Task {
            name: self.name.clone(),
            group_name: self.group_name.clone(),
            command: self.command.clone(),
            args,
            working_dir: self.working_dir.clone(),
            env_vars,
        })
    }
}

/// 任务组实体转换为业务模型的方法
impl TaskGroupEntity {
    pub fn to_task_group(&self) -> core::TaskGroup {
        core::TaskGroup {
            name: self.name.clone(),
            working_dir: self.working_dir.clone(),
            tasks: Vec::new(), // 需要单独查询
        }
    }
}

/// 业务模型转换为数据库实体的方法
impl From<&core::TaskGroup> for TaskGroupEntity {
    fn from(task_group: &core::TaskGroup) -> Self {
        TaskGroupEntity {
            name: task_group.name.clone(),
            working_dir: task_group.working_dir.clone(),
            status: "created".to_string(),
            created_at: chrono::Utc::now().to_rfc3339(),
            deleted_at: None,
            started_at: None,
            completed_at: None,
        }
    }
}

impl From<&core::Task> for TaskEntity {
    fn from(task: &core::Task) -> Self {
        TaskEntity {
            group_name: task.group_name.clone(),
            name: task.name.clone(),
            command: task.command.clone(),
            args: task
                .args
                .as_ref()
                .map(|v| serde_json::to_string(v).unwrap()),
            working_dir: task.working_dir.clone(),
            env_vars: task
                .env_vars
                .as_ref()
                .map(|v| serde_json::to_string(v).unwrap()),
            status: "created".to_string(),
            error_message: None,
            created_at: chrono::Utc::now().to_rfc3339(),
            deleted_at: None,
            started_at: None,
            completed_at: None,
        }
    }
}
