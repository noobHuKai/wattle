use core::{Worker, Workflow, WorkerStatus};
use std::collections::HashMap;

#[test]
fn test_worker_creation() {
    let worker = Worker {

        name: "test_worker".to_string(),
        workflow_name: "test_workflow".to_string(),
        command: "echo hello".to_string(),
        args: Some(vec!["world".to_string()]),
        working_dir: Some("/tmp".to_string()),
        env_vars: Some({
            let mut map = HashMap::new();
            map.insert("TEST_VAR".to_string(), "test_value".to_string());
            map
        }),
        inputs: None,
        outputs: None,
    };

    assert_eq!(worker.name, "test_worker");
    assert_eq!(worker.workflow_name, "test_workflow");
    assert_eq!(worker.command, "echo hello");
    assert!(worker.args.is_some());
    assert!(worker.working_dir.is_some());
    assert!(worker.env_vars.is_some());
}

#[test]
fn test_workflow_creation() {
    let worker = Worker {
        name: "worker1".to_string(),
        workflow_name: "test_workflow".to_string(),
        command: "echo test".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
        inputs: None,
        outputs: None,
    };

    let workflow = Workflow {
        name: "test_workflow".to_string(),
        working_dir: Some("/home/test".to_string()),
        workers: vec![worker],
    };

    assert_eq!(workflow.name, "test_workflow");
    assert_eq!(workflow.workers.len(), 1);
    assert_eq!(workflow.workers[0].name, "worker1");
}

#[test]
fn test_worker_status_display() {
    assert_eq!(WorkerStatus::Created.to_string(), "created");
    assert_eq!(WorkerStatus::Running.to_string(), "running");
    assert_eq!(WorkerStatus::Completed.to_string(), "completed");
    assert_eq!(WorkerStatus::Failed.to_string(), "failed");
    assert_eq!(WorkerStatus::Cancelled.to_string(), "cancelled");
}

#[test]
fn test_worker_status_equality() {
    assert_eq!(WorkerStatus::Created, WorkerStatus::Created);
    assert_ne!(WorkerStatus::Created, WorkerStatus::Running);
}

#[test]
fn test_default_values() {
    let worker = Worker::default();
    assert_eq!(worker.name, "");
    assert_eq!(worker.workflow_name, "");
    assert_eq!(worker.command, "");

    let workflow = Workflow::default();
    assert_eq!(workflow.name, "");
    assert!(workflow.workers.is_empty());

    let status = WorkerStatus::default();
    assert_eq!(status, WorkerStatus::Created);
}

#[test]
fn test_serialization() {
    let worker = Worker {
        name: "test_worker".to_string(),
        workflow_name: "test_workflow".to_string(),
        command: "echo hello".to_string(),
        args: Some(vec!["world".to_string()]),
        working_dir: Some("/tmp".to_string()),
        env_vars: None,
        inputs: None,
        outputs: None,
    };

    // Test JSON serialization
    let json = serde_json::to_string(&worker).unwrap();
    assert!(json.contains("test_worker"));
    assert!(json.contains("test_workflow"));

    // Test deserialization
    let deserialized: Worker = serde_json::from_str(&json).unwrap();
    assert_eq!(deserialized.name, worker.name);
    assert_eq!(deserialized.workflow_name, worker.workflow_name);
}
