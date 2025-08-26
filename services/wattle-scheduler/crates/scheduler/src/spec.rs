use core::Task;
use std::collections::HashMap;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskGroupSpec {
    pub name: Option<String>,                // 可选，默认为 "default"
    pub services: HashMap<String, TaskSpec>, // 默认为空集合
    pub current_dir: Option<String>,         // 当前目录，用于相对路径
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskSpec {
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

impl Into<Task> for TaskSpec {
    fn into(self) -> Task {
        Task {
            name: self.name.unwrap_or_default(),
            args: self.args,
            working_dir: self.working_dir,
            env_vars: self.env_vars,
            group_name: "".to_string(),
            command: self.cmd,
        }
    }
}
