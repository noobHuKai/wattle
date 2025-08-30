use coordinator::{Coordinator, CoordinatorConfig};
use core::ExecutionConfig;
use std::time::Duration;

// We can't easily import from the binary crate, so we'll test at a higher level
async fn create_test_coordinator() -> Coordinator {
    let temp_dir = tempfile::tempdir().unwrap();
    let config = CoordinatorConfig {
        db_url: Some(temp_dir.path().join("test.db").to_string_lossy().to_string()),
        execution: Some(ExecutionConfig {
            log_dir: Some(temp_dir.path().to_path_buf()),
            timeout_secs: Some(30),
            max_parallel_tasks: Some(4),
        }),
    };
    
    Coordinator::new(config).await.unwrap()
}

#[tokio::test]
async fn test_coordinator_integration() {
    let coordinator = create_test_coordinator().await;
    
    // Test basic coordinator functionality that the API server would use
    let workflows = coordinator.list_workflows().await.unwrap();
    assert!(workflows.is_empty());
}

#[tokio::test]
async fn test_coordinator_workflow_operations() {
    use coordinator::{Workflow, Worker};
    use std::collections::HashMap;
    
    let coordinator = create_test_coordinator().await;
    
    // Create test workflow
    let mut env_vars = HashMap::new();
    env_vars.insert("TEST_ENV".to_string(), "test_value".to_string());
    
    let worker = Worker {

        name: "test-worker".to_string(),
        workflow_name: "api-test-workflow".to_string(),
        command: "echo test".to_string(),
        args: Some(vec!["hello".to_string()]),
        working_dir: None,
        env_vars: Some(env_vars),
        inputs: None,
        outputs: None,
    };
    
    let workflow = Workflow {
        name: "api-test-workflow".to_string(),
        working_dir: None,
        workers: vec![worker],
    };
    
    // Test workflow creation
    let result = coordinator.create_workflow(workflow).await;
    assert!(result.is_ok());
    
    // Test workflow listing
    let workflows = coordinator.list_workflows().await.unwrap();
    assert_eq!(workflows.len(), 1);
    assert_eq!(workflows[0].name, "api-test-workflow");
    
    // Test workflow existence
    let exists = coordinator.workflow_exists("api-test-workflow").await.unwrap();
    assert!(exists);
    
    let not_exists = coordinator.workflow_exists("nonexistent").await.unwrap();
    assert!(!not_exists);
}

#[tokio::test]
async fn test_coordinator_worker_operations() {
    use coordinator::{Workflow, Worker};
    
    let coordinator = create_test_coordinator().await;
    
    // Create workflow with multiple workers
    let workers = vec![
        Worker {

            name: "worker1".to_string(),
            workflow_name: "multi-worker-test".to_string(),
            command: "echo worker1".to_string(),
            args: None,
            working_dir: None,
            env_vars: None,
        inputs: None,
        outputs: None,
    },
        Worker {

            name: "worker2".to_string(),
            workflow_name: "multi-worker-test".to_string(),
            command: "echo worker2".to_string(),
            args: None,
            working_dir: None,
            env_vars: None,
        inputs: None,
        outputs: None,
    },
    ];
    
    let workflow = Workflow {
        name: "multi-worker-test".to_string(),
        working_dir: None,
        workers,
    };
    
    coordinator.create_workflow(workflow).await.unwrap();
    
    // Test worker listing
    let workers = coordinator.list_workers("multi-worker-test").await.unwrap();
    assert_eq!(workers.len(), 2);
    
    // Test worker existence
    let exists = coordinator.worker_exists("multi-worker-test", "worker1").await.unwrap();
    assert!(exists);
    
    let not_exists = coordinator.worker_exists("multi-worker-test", "nonexistent").await.unwrap();
    assert!(!not_exists);
}

#[test]
fn test_basic_functionality() {
    // Test basic structures and functions that don't require async
    use std::net::SocketAddr;
    
    // Test address parsing
    let addr: SocketAddr = "127.0.0.1:3000".parse().unwrap();
    assert_eq!(addr.ip().to_string(), "127.0.0.1");
    assert_eq!(addr.port(), 3000);
    
    // Test JSON operations
    let json_data = serde_json::json!({
        "workflows": [],
        "status": "healthy"
    });
    
    assert!(json_data["workflows"].is_array());
    assert_eq!(json_data["status"], "healthy");
}
