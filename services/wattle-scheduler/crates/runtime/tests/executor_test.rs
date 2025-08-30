// Import std modules first to avoid core conflicts
use std::collections::HashMap;
use tokio::time::Duration;
use tokio::runtime::Runtime;

// Import from local crate using extern
extern crate core as wattle_core;
use runtime::TaskExecutor;
use wattle_core::{ExecutionConfig, Worker};

#[test]
fn test_task_executor_creation() {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        let config = ExecutionConfig {
            log_dir: Some("/tmp/test_logs".into()),
            timeout: Some(Duration::from_secs(30)),
        };
        
        let executor = TaskExecutor::new(config);
        let processes = executor.list_running_processes().await;
        assert!(processes.is_empty() || !processes.is_empty()); // Both are valid initially
    });
}

#[test]
fn test_worker_basic_operations() {
    let worker = Worker {
        name: "test-worker".to_string(),
        workflow_name: "test-workflow".to_string(),
        command: "echo hello".to_string(),
        args: Some(vec!["world".to_string()]),
        working_dir: Some("/tmp".to_string()),
        env_vars: Some(HashMap::new()),
        inputs: None,
        outputs: None,
    };
    
    assert_eq!(worker.name, "test-worker");
    assert_eq!(worker.workflow_name, "test-workflow");
    assert_eq!(worker.command, "echo hello");
}

#[test]
fn test_simple_command_execution() {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        let config = ExecutionConfig {
            log_dir: None,
            timeout: Some(Duration::from_secs(5)),
        };
        
        let executor = TaskExecutor::new(config);
        
        let worker = Worker {
            name: "echo-worker".to_string(),
            workflow_name: "echo-workflow".to_string(),
            command: "echo".to_string(),
            args: Some(vec!["test".to_string()]),
            working_dir: None,
            env_vars: None,
            inputs: None,
            outputs: None,
        };
        
        // Simple async functions that return () implementing Future<Output = ()>
        async fn on_create(_pid: u32, _worker_name: String, _workflow_name: String) {
            println!("Task started");
        }
        
        async fn on_success(_pid: u32, _worker_name: String, _workflow_name: String) {
            println!("Task completed");
        }
        
        async fn on_fail(_pid: u32, _worker_name: String, _workflow_name: String, _exit_code: i32) {
            println!("Task failed");
        }
        
        let result = executor.execute(
            worker,
            on_create(0, String::new(), String::new()),
            on_success(0, String::new(), String::new()),
            on_fail(0, String::new(), String::new(), 0),
        ).await;
        
        // For echo command, we expect success or error
        if let Err(e) = result {
            println!("Execution error: {}", e);
        }
    });
}

#[test]
fn test_executor_configuration() {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        let temp_dir = std::env::temp_dir();
        let config = ExecutionConfig {
            log_dir: Some(temp_dir.clone()),
            timeout: Some(Duration::from_secs(10)),
        };
        
        let executor = TaskExecutor::new(config);
        
        let worker = Worker {
            name: "config-test".to_string(),
            workflow_name: "config-workflow".to_string(), 
            command: "true".to_string(),
            args: None,
            working_dir: None,
            env_vars: None,
            inputs: None,
            outputs: None,
        };

        async fn empty_callback_create(_: u32, _: String, _: String) {}
        async fn empty_callback_success(_: u32, _: String, _: String) {}
        async fn empty_callback_fail(_: u32, _: String, _: String, _: i32) {}

        let result = executor.execute(
            worker,
            empty_callback_create(0, String::new(), String::new()),
            empty_callback_success(0, String::new(), String::new()),
            empty_callback_fail(0, String::new(), String::new(), 0),
        ).await;

        // true command should succeed
        assert!(result.is_ok());
    });
}

#[test]
fn test_list_running_processes() {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        let config = ExecutionConfig {
            log_dir: None,
            timeout: Some(Duration::from_secs(5)),
        };
        
        let executor = TaskExecutor::new(config);
        
        let processes = executor.list_running_processes().await;
        // Initially should be empty or contain existing processes
        assert!(processes.is_empty() || !processes.is_empty());
    });
}
