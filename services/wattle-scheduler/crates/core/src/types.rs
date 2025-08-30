use std::collections::HashMap;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Worker {
    pub name: String,
    pub workflow_name: String,
    pub command: String,
    pub args: Option<Vec<String>>,
    pub working_dir: Option<String>,
    pub env_vars: Option<HashMap<String, String>>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Workflow {
    pub name: String,
    pub working_dir: Option<String>,
    pub workers: Vec<Worker>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq, Eq, Hash)]
pub enum WorkerStatus {
    #[default]
    Created,
    Running,
    Completed,
    Failed,
    Cancelled,
}
impl std::fmt::Display for WorkerStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let status_str = match self {
            WorkerStatus::Created => "created",
            WorkerStatus::Running => "running",
            WorkerStatus::Completed => "completed",
            WorkerStatus::Failed => "failed",
            WorkerStatus::Cancelled => "cancelled",
        };
        write!(f, "{}", status_str)
    }
}
