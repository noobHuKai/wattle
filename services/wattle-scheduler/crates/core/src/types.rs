use std::collections::HashMap;

use serde::{Deserialize, Serialize};
use petgraph::{Graph, Direction};
use petgraph::algo::toposort;
use eyre::{Result, eyre};
use thiserror::Error;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Worker {
    pub name: String,
    pub workflow_name: String,
    pub command: String,
    pub args: Option<Vec<String>>,
    pub working_dir: Option<String>,
    pub env_vars: Option<HashMap<String, String>>,
    pub inputs: Option<HashMap<String, String>>,
    pub outputs: Option<HashMap<String, String>>,
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

#[derive(Error, Debug)]
pub enum WorkflowError {
    #[error("Dependency not found: worker '{worker}' depends on '{dependency}' but no worker outputs this key")]
    DependencyNotFound { worker: String, dependency: String },
    #[error("Circular dependency detected in workflow")]
    CircularDependency,
    #[error("Invalid dependency format: '{dependency}' should be in format 'worker_name/output_key'")]
    InvalidDependencyFormat { dependency: String },
}

impl Workflow {
    /// 验证工作流的依赖关系是否合法
    pub fn validate_dependencies(&self) -> Result<()> {
        // 收集所有 worker 的 outputs
        let mut all_outputs: HashMap<String, Vec<String>> = HashMap::new();
        
        for worker in &self.workers {
            if let Some(ref outputs) = worker.outputs {
                all_outputs.insert(worker.name.clone(), outputs.keys().cloned().collect());
            }
        }
        
        // 检查每个 worker 的 inputs 依赖
        for worker in &self.workers {
            if let Some(ref inputs) = worker.inputs {
                for (_input_key, dependency_path) in inputs {
                    // 解析依赖路径 (格式: worker_name/output_key)
                    let parts: Vec<&str> = dependency_path.split('/').collect();
                    if parts.len() < 2 {
                        return Err(eyre!(WorkflowError::InvalidDependencyFormat { 
                            dependency: dependency_path.clone() 
                        }));
                    }
                    
                    let dependency_worker = parts[0];
                    let dependency_output = parts[1];
                    
                    // 检查依赖的 worker 是否存在且有对应的 output
                    if let Some(outputs) = all_outputs.get(dependency_worker) {
                        if !outputs.contains(&dependency_output.to_string()) {
                            return Err(eyre!(WorkflowError::DependencyNotFound {
                                worker: worker.name.clone(),
                                dependency: format!("{}/{}", dependency_worker, dependency_output)
                            }));
                        }
                    } else {
                        return Err(eyre!(WorkflowError::DependencyNotFound {
                            worker: worker.name.clone(),
                            dependency: format!("{}/{}", dependency_worker, dependency_output)
                        }));
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// 构建依赖图并返回拓扑排序后的执行顺序
    pub fn get_execution_order(&self) -> Result<Vec<Vec<String>>> {
        // 创建有向图
        let mut graph = Graph::new();
        let mut node_indices = HashMap::new();
        
        // 添加所有 worker 作为节点
        for worker in &self.workers {
            let node_index = graph.add_node(worker.name.clone());
            node_indices.insert(worker.name.clone(), node_index);
        }
        
        // 添加依赖边 (如果 A 依赖 B，则添加 B -> A 的边)
        for worker in &self.workers {
            if let Some(ref inputs) = worker.inputs {
                for (_, dependency_path) in inputs {
                    let parts: Vec<&str> = dependency_path.split('/').collect();
                    if parts.len() >= 2 {
                        let dependency_worker = parts[0];
                        
                        if let (Some(&from_idx), Some(&to_idx)) = (
                            node_indices.get(dependency_worker),
                            node_indices.get(&worker.name)
                        ) {
                            // 添加从依赖 worker 到当前 worker 的边
                            graph.add_edge(from_idx, to_idx, ());
                        }
                    }
                }
            }
        }
        
        // 拓扑排序
        let topo_result = toposort(&graph, None);
        let sorted_nodes = match topo_result {
            Ok(nodes) => nodes,
            Err(_) => return Err(eyre!(WorkflowError::CircularDependency)),
        };
        
        // 使用 Khan's 算法来分层
        let mut execution_levels: Vec<Vec<String>> = Vec::new();
        let mut in_degree: HashMap<_, usize> = HashMap::new();
        
        // 计算每个节点的入度
        for &node_idx in &sorted_nodes {
            in_degree.insert(node_idx, graph.neighbors_directed(node_idx, Direction::Incoming).count());
        }
        
        let mut remaining_nodes: std::collections::HashSet<_> = sorted_nodes.into_iter().collect();
        
        while !remaining_nodes.is_empty() {
            let mut current_level = Vec::new();
            
            // 找出入度为0的节点
            let nodes_with_zero_indegree: Vec<_> = remaining_nodes
                .iter()
                .filter(|&node_idx| in_degree.get(node_idx).unwrap_or(&0) == &0)
                .cloned()
                .collect();
            
            if nodes_with_zero_indegree.is_empty() && !remaining_nodes.is_empty() {
                return Err(eyre!(WorkflowError::CircularDependency));
            }
            
            // 将这些节点加入当前层级
            for node_idx in nodes_with_zero_indegree {
                let worker_name = &graph[node_idx];
                current_level.push(worker_name.clone());
                remaining_nodes.remove(&node_idx);
                
                // 更新所有邻居的入度
                for neighbor in graph.neighbors_directed(node_idx, Direction::Outgoing) {
                    if let Some(degree) = in_degree.get_mut(&neighbor) {
                        *degree = degree.saturating_sub(1);
                    }
                }
            }
            
            if !current_level.is_empty() {
                execution_levels.push(current_level);
            }
        }
        
        Ok(execution_levels)
    }
}
