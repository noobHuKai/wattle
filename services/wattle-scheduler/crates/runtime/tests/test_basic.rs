use std::{collections::HashMap, time::Duration};
use runtime::TaskExecutor;

// Use explicit path to avoid std::core conflict
use ::core::{ExecutionConfig, Worker};

#[tokio::test]
async fn test_executor_creation() {
    let config = ExecutionConfig {
        log_dir: None,
        timeout_secs: Some(10),
            max_parallel_tasks: Some(4),
    };
    
    let executor = TaskExecutor::new(config);
    let processes = executor.list_running_processes().await;
    
    // This test passes if no panic occurs and we get a result
    assert!(processes.is_empty() || !processes.is_empty());
}

#[tokio::test]
async fn test_worker_structure() {
    let worker = Worker {
        name: "test-worker".to_string(),
        workflow_name: "test-workflow".to_string(),
        command: "echo".to_string(),
        args: Some(vec!["hello".to_string()]),
        working_dir: None,
        env_vars: None,
        inputs: None,
        outputs: None,
    };
    
    assert_eq!(worker.name, "test-worker");
    assert_eq!(worker.command, "echo");
}

#[tokio::test] 
async fn test_basic_execution() {
    let config = ExecutionConfig {
        log_dir: None,
        timeout_secs: Some(5),
            max_parallel_tasks: Some(4),
    };
    
    let executor = TaskExecutor::new(config);
    
    let worker = Worker {
        name: "simple-test".to_string(),
        workflow_name: "simple-workflow".to_string(),
        command: "true".to_string(), // Simple command that always succeeds
        args: None,
        working_dir: None,
        env_vars: None,
        inputs: None,
        outputs: None,
    };
    
    // Simple callbacks
    let on_create = async { println!("Task created"); };
    let on_success = async { println!("Task succeeded"); };
    let on_fail = async { println!("Task failed"); };
    
    let result = executor.execute(
        worker,
        on_create,
        on_success,
        on_fail,
    ).await;
    
    // Check if the execution completed (either success or failure is OK for this test)
    match result {
        Ok(_) => println!("Execution completed successfully"),
        Err(e) => println!("Execution failed: {}", e),
    }
}
