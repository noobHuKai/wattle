use core::Worker;
use std::collections::HashMap;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkflowSpec {
    pub name: Option<String>,                // 可选，默认为 "default"
    pub services: HashMap<String, WorkerSpec>, // 默认为空集合
    pub current_dir: Option<String>,         // 当前目录，用于相对路径
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkerSpec {
    pub name: Option<String>,
    pub cmd: String,
    pub args: Option<Vec<String>>,
    pub working_dir: Option<String>,
    pub env_vars: Option<HashMap<String, String>>,
    pub timeout: Option<u64>,
    pub max_retries: Option<u32>,
    pub input: Option<HashMap<String, String>>,
    pub output: Option<HashMap<String, String>>,
}

impl Into<Worker> for WorkerSpec {
    fn into(self) -> Worker {
        Worker {
            name: self.name.unwrap_or_default(),
            args: self.args,
            working_dir: self.working_dir,
            env_vars: self.env_vars,
            workflow_name: "".to_string(),
            command: self.cmd,
        }
    }
}

// API响应结构体
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkerSummary {
    pub name: String,
    pub workflow_name: String,
    pub status: String,
    pub command: String,
    pub created_at: String,
    pub started_at: Option<String>,
    pub completed_at: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkerInfo {
    pub name: String,
    pub workflow_name: String,
    pub command: String,
    pub args: Option<Vec<String>>,
    pub working_dir: Option<String>,
    pub env_vars: Option<std::collections::HashMap<String, String>>,
    pub status: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkflowSummary {
    pub name: String,
    pub working_dir: Option<String>,
    pub status: String,
    pub created_at: String,
}
