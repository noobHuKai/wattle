use coordinator::{Coordinator, CoordinatorConfig, Workflow, Worker};
use std::time::Duration;
use tokio::runtime::Runtime;

// Import from local crate using extern to avoid conflicts 
extern crate core as wattle_core;
use wattle_core::ExecutionConfig;

#[test]
fn test_coordinator_creation() {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        let temp_dir = tempfile::tempdir().unwrap();
        let config = CoordinatorConfig {
            db_url: Some(temp_dir.path().join("test.db").to_string_lossy().to_string()),
            execution: Some(ExecutionConfig {
                log_dir: Some(temp_dir.path().to_path_buf()),
                timeout: Some(Duration::from_secs(30)),
            }),
        };
        
        let result = Coordinator::new(config).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_workflow_creation_and_retrieval() {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        let temp_dir = tempfile::tempdir().unwrap();
        let config = CoordinatorConfig {
            db_url: Some(temp_dir.path().join("test.db").to_string_lossy().to_string()),
            execution: Some(ExecutionConfig {
                log_dir: Some(temp_dir.path().to_path_buf()),
                timeout: Some(Duration::from_secs(30)),
            }),
        };
        
        let coordinator = Coordinator::new(config).await.unwrap();
        
        // Create a test workflow
        let worker = Worker {
            name: "test-worker".to_string(),
            workflow_name: "test-workflow".to_string(),
            command: "echo test".to_string(),
            args: None,
            working_dir: None,
            env_vars: None,
            inputs: None,
            outputs: None,
        };
        
        let workflow = Workflow {
            name: "test-workflow".to_string(),
            working_dir: Some("/tmp".to_string()),
            workers: vec![worker],
        };
        
        // Create workflow
        let result = coordinator.create_workflow(workflow).await;
        assert!(result.is_ok());
        
        // List workflows
        let workflows = coordinator.list_workflows().await.unwrap();
        assert!(!workflows.is_empty());
        assert_eq!(workflows[0].name, "test-workflow");
    });
}

#[test]
fn test_workflow_execution() {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        let temp_dir = tempfile::tempdir().unwrap();
        let config = CoordinatorConfig {
            db_url: Some(temp_dir.path().join("test.db").to_string_lossy().to_string()),
            execution: Some(ExecutionConfig {
                log_dir: Some(temp_dir.path().to_path_buf()),
                timeout: Some(Duration::from_secs(30)),
            }),
        };
        
        let coordinator = Coordinator::new(config).await.unwrap();
        
        // Create and submit workflow
        let worker = Worker {
            name: "echo-worker".to_string(),
            workflow_name: "echo-workflow".to_string(),
            command: "echo 'Hello from test'".to_string(),
            args: None,
            working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
            env_vars: None,
            inputs: None,
            outputs: None,
        };
        
        let workflow = Workflow {
            name: "echo-workflow".to_string(),
            working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
            workers: vec![worker],
        };
        
        coordinator.create_workflow(workflow).await.unwrap();
        
        // Execute workflow
        let result = coordinator.run_workflow(&"echo-workflow".to_string()).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_workflow_not_found() {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        let temp_dir = tempfile::tempdir().unwrap();
        let config = CoordinatorConfig {
            db_url: Some(temp_dir.path().join("test.db").to_string_lossy().to_string()),
            execution: Some(ExecutionConfig {
                log_dir: Some(temp_dir.path().to_path_buf()),
                timeout: Some(Duration::from_secs(30)),
            }),
        };
        
        let coordinator = Coordinator::new(config).await.unwrap();
        
        // Check if workflow exists
        let exists = coordinator.workflow_exists("non-existent").await.unwrap();
        assert!(!exists);
    });
}

#[test]
fn test_list_empty_workflows() {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        let temp_dir = tempfile::tempdir().unwrap();
        let config = CoordinatorConfig {
            db_url: Some(temp_dir.path().join("test.db").to_string_lossy().to_string()),
            execution: Some(ExecutionConfig {
                log_dir: Some(temp_dir.path().to_path_buf()),
                timeout: Some(Duration::from_secs(30)),
            }),
        };
        
        let coordinator = Coordinator::new(config).await.unwrap();
        
        let workflows = coordinator.list_workflows().await.unwrap();
        assert!(workflows.is_empty());
    });
}
