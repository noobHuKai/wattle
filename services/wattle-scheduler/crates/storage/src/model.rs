use core::{Worker, Workflow};

use serde::{Deserialize, Serialize};
use sqlx::FromRow;

/// 数据库中的工作流实体
#[derive(Debug, Clone, Serialize, Deserialize, FromRow)]
pub struct WorkflowEntity {
    pub name: String,
    pub working_dir: Option<String>,
    pub status: String,
    pub created_at: String, // SQLite 存储为 TEXT
    pub deleted_at: Option<String>,
    pub started_at: Option<String>,
    pub completed_at: Option<String>,
}

impl Into<Workflow> for WorkflowEntity {
    fn into(self) -> Workflow {
        Workflow {
            name: self.name,
            working_dir: self.working_dir,
            workers: Vec::new(), // 需要单独查询
        }
    }
}

/// 数据库中的工作者实体
#[derive(Debug, Clone, Serialize, Deserialize, FromRow)]
pub struct WorkerEntity {
    pub workflow_name: String,
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

impl Into<Worker> for WorkerEntity {
    fn into(self) -> Worker {
        Worker {
            workflow_name: self.workflow_name,
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

/// 数据库中的工作者日志实体
#[derive(Debug, Clone, Serialize, Deserialize, FromRow)]
pub struct WorkerLogEntity {
    pub id: i64,
    pub workflow_name: String,
    pub worker_name: String,
    pub log_type: String,
    pub file_path: String,
    pub created_at: String,
}

/// 工作者实体转换为业务模型的方法
impl WorkerEntity {
    pub fn to_worker(&self) -> Result<core::Worker, serde_json::Error> {
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

        Ok(core::Worker {
            name: self.name.clone(),
            workflow_name: self.workflow_name.clone(),
            command: self.command.clone(),
            args,
            working_dir: self.working_dir.clone(),
            env_vars,
        })
    }

    pub fn get_input_schema(&self) -> Option<std::collections::HashMap<String, serde_json::Value>> {
        // 从数据库或其他地方获取输入 schema
        // 这里是示例实现
        None
    }

    pub fn get_output_schema(&self) -> Option<std::collections::HashMap<String, serde_json::Value>> {
        // 从数据库或其他地方获取输出 schema  
        // 这里是示例实现
        None
    }
}

/// 工作流实体转换为业务模型的方法
impl WorkflowEntity {
    pub fn to_workflow(&self, workers: Vec<WorkerEntity>) -> Result<core::Workflow, serde_json::Error> {
        let mut worker_list = Vec::new();
        for worker_entity in workers {
            worker_list.push(worker_entity.to_worker()?);
        }

        Ok(core::Workflow {
            name: self.name.clone(),
            working_dir: self.working_dir.clone(),
            workers: worker_list,
        })
    }
}

/// 业务模型转换为数据库实体的方法
impl From<&core::Workflow> for WorkflowEntity {
    fn from(workflow: &core::Workflow) -> Self {
        WorkflowEntity {
            name: workflow.name.clone(),
            working_dir: workflow.working_dir.clone(),
            status: "created".to_string(),
            created_at: chrono::Utc::now().to_rfc3339(),
            deleted_at: None,
            started_at: None,
            completed_at: None,
        }
    }
}

impl From<&core::Worker> for WorkerEntity {
    fn from(worker: &core::Worker) -> Self {
        WorkerEntity {
            workflow_name: worker.workflow_name.clone(),
            name: worker.name.clone(),
            command: worker.command.clone(),
            args: worker.args.as_ref().map(|args| serde_json::to_string(args).unwrap_or_default()),
            working_dir: worker.working_dir.clone(),
            env_vars: worker.env_vars.as_ref().map(|env| serde_json::to_string(env).unwrap_or_default()),
            status: "created".to_string(),
            error_message: None,
            created_at: chrono::Utc::now().to_rfc3339(),
            deleted_at: None,
            started_at: None,
            completed_at: None,
        }
    }
}
