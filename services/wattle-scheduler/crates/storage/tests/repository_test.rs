use storage::{init_database, Repositories};
use core::{Workflow, Worker, WorkerStatus};

#[tokio::test]
async fn test_database_initialization() {
    let temp_dir = tempfile::tempdir().unwrap();
    let db_path = temp_dir.path().join("test.db");
    
    let db = init_database(Some(db_path.to_string_lossy().to_string())).await;
    assert!(db.is_ok());
}

#[tokio::test]
async fn test_in_memory_database() {
    let db = init_database(None).await;
    assert!(db.is_ok());
}

#[tokio::test]
async fn test_workflow_operations() {
    let temp_dir = tempfile::tempdir().unwrap();
    let db_path = temp_dir.path().join("test.db");
    
    let repos = Repositories::new(db_path.to_string_lossy().to_string()).await.unwrap();
    
    // Create a test workflow
    let workflow = Workflow {
        name: "test-workflow".to_string(),
        working_dir: Some("/tmp".to_string()),
        workers: vec![],
    };
    
    // Insert workflow
    let result = repos.insert_workflow(&workflow).await;
    assert!(result.is_ok());
    
    // Get workflow by name
    let found = repos.get_workflow("test-workflow").await.unwrap();
    assert!(found.is_some());
    assert_eq!(found.unwrap().name, "test-workflow");
    
    // List all workflows
    let workflows = repos.get_all_workflows().await.unwrap();
    assert_eq!(workflows.len(), 1);
    
    // Check if workflow exists
    let exists = repos.workflow_exists("test-workflow").await.unwrap();
    assert!(exists);
    
    let not_exists = repos.workflow_exists("non-existent").await.unwrap();
    assert!(!not_exists);
}

#[tokio::test]
async fn test_worker_operations() {
    let temp_dir = tempfile::tempdir().unwrap();
    let db_path = temp_dir.path().join("test.db");
    
    let repos = Repositories::new(db_path.to_string_lossy().to_string()).await.unwrap();
    
    // First create a workflow
    let workflow = Workflow {
        name: "test-workflow".to_string(),
        working_dir: Some("/tmp".to_string()),
        workers: vec![],
    };
    repos.insert_workflow(&workflow).await.unwrap();
    
    // Create a test worker
    let worker = Worker {
        name: "test-worker".to_string(),
        workflow_name: "test-workflow".to_string(),
        command: "echo test".to_string(),
        args: Some(vec!["hello".to_string()]),
        working_dir: Some("/tmp".to_string()),
        env_vars: None,
    };
    
    // Insert worker
    let result = repos.insert_worker(&worker).await;
    assert!(result.is_ok());
    
    // Find worker by workflow and name
    let found = repos.get_worker("test-workflow", "test-worker").await.unwrap();
    assert!(found.is_some());
    let found_worker = found.unwrap();
    assert_eq!(found_worker.name, "test-worker");
    assert_eq!(found_worker.workflow_name, "test-workflow");
    
    // List workers by workflow
    let workers = repos.get_workers_by_workflow("test-workflow").await.unwrap();
    assert_eq!(workers.len(), 1);
    
    // Update worker status
    let result = repos.update_worker_status("test-workflow", "test-worker", WorkerStatus::Running).await;
    assert!(result.is_ok());
    
    // Verify status update
    let updated = repos.get_worker("test-workflow", "test-worker").await.unwrap().unwrap();
    assert_eq!(updated.status, WorkerStatus::Running);
}

#[tokio::test]
async fn test_complex_workflow() {
    let temp_dir = tempfile::tempdir().unwrap();
    let db_path = temp_dir.path().join("test.db");
    
    let repos = Repositories::new(db_path.to_string_lossy().to_string()).await.unwrap();
    
    // Create workers
    let worker1 = Worker {
        name: "worker1".to_string(),
        workflow_name: "complex-workflow".to_string(),
        command: "echo worker1".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
    };
    
    let worker2 = Worker {
        name: "worker2".to_string(),
        workflow_name: "complex-workflow".to_string(),
        command: "echo worker2".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
    };
    
    // Create workflow with workers
    let workflow = Workflow {
        name: "complex-workflow".to_string(),
        working_dir: Some("/tmp".to_string()),
        workers: vec![worker1, worker2],
    };
    
    // Insert workflow
    repos.insert_workflow(&workflow).await.unwrap();
    
    // Insert workers
    for worker in &workflow.workers {
        repos.insert_worker(worker).await.unwrap();
    }
    
    // Verify everything was created
    let workflow_found = repos.get_workflow("complex-workflow").await.unwrap();
    assert!(workflow_found.is_some());
    
    let workers_found = repos.get_workers_by_workflow("complex-workflow").await.unwrap();
    assert_eq!(workers_found.len(), 2);
    
    // Verify worker names
    let worker_names: Vec<String> = workers_found.iter().map(|w| w.name.clone()).collect();
    assert!(worker_names.contains(&"worker1".to_string()));
    assert!(worker_names.contains(&"worker2".to_string()));
}

#[tokio::test]
async fn test_worker_status_transitions() {
    let temp_dir = tempfile::tempdir().unwrap();
    let db_path = temp_dir.path().join("test.db");
    
    let repos = Repositories::new(db_path.to_string_lossy().to_string()).await.unwrap();
    
    // Create workflow and worker
    let workflow = Workflow {
        name: "status-test".to_string(),
        working_dir: None,
        workers: vec![],
    };
    repos.insert_workflow(&workflow).await.unwrap();
    
    let worker = Worker {
        name: "status-worker".to_string(),
        workflow_name: "status-test".to_string(),
        command: "echo test".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
    };
    repos.insert_worker(&worker).await.unwrap();
    
    // Test status transitions
    let statuses = vec![
        WorkerStatus::Running,
        WorkerStatus::Completed,
        WorkerStatus::Failed,
        WorkerStatus::Cancelled,
    ];
    
    for status in statuses {
        let result = repos.update_worker_status("status-test", "status-worker", status.clone()).await;
        assert!(result.is_ok());
        
        let updated = repos.get_worker("status-test", "status-worker").await.unwrap().unwrap();
        assert_eq!(updated.status, status);
    }
}
