use std::collections::HashMap;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Task {
    pub name: String,
    pub group_name: String,
    pub command: String,
    pub args: Option<Vec<String>>,
    pub working_dir: Option<String>,
    pub env_vars: Option<HashMap<String, String>>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct TaskGroup {
    pub name: String,
    pub working_dir: Option<String>,
    pub tasks: Vec<Task>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq, Eq)]
pub enum TaskStatus {
    #[default]
    Created,
    Running,
    Completed,
    Failed,
    Cancelled,
}
impl std::fmt::Display for TaskStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let status_str = match self {
            TaskStatus::Created => "created",
            TaskStatus::Running => "running",
            TaskStatus::Completed => "completed",
            TaskStatus::Failed => "failed",
            TaskStatus::Cancelled => "cancelled",
        };
        write!(f, "{}", status_str)
    }
}
