use std::collections::HashMap;
use core as wattle_core;
use wattle_core::{Worker, Workflow};

#[test]
fn test_workflow_with_dag_dependencies() {
    // 创建一个测试工作流，演示 DAG 功能
    let mut worker1_outputs = HashMap::new();
    worker1_outputs.insert("processed_data".to_string(), "/tmp/processed_data.json".to_string());
    
    let worker1 = Worker {
        name: "data_processor".to_string(),
        workflow_name: "dag_test".to_string(),
        command: "echo".to_string(),
        args: Some(vec!["Processing data".to_string()]),
        working_dir: Some("/tmp".to_string()),
        env_vars: None,
        inputs: None,
        outputs: Some(worker1_outputs),
    };
    
    let mut worker2_inputs = HashMap::new();
    worker2_inputs.insert("data".to_string(), "data_processor/processed_data".to_string());
    
    let mut worker2_outputs = HashMap::new();
    worker2_outputs.insert("analysis_result".to_string(), "/tmp/analysis.json".to_string());
    
    let worker2 = Worker {
        name: "analyzer".to_string(),
        workflow_name: "dag_test".to_string(),
        command: "echo".to_string(),
        args: Some(vec!["Analyzing data".to_string()]),
        working_dir: Some("/tmp".to_string()),
        env_vars: None,
        inputs: Some(worker2_inputs),
        outputs: Some(worker2_outputs),
    };
    
    let workflow = Workflow {
        name: "dag_test".to_string(),
        working_dir: Some("/tmp".to_string()),
        workers: vec![worker1, worker2],
    };
    
    // 验证依赖关系
    let validation_result = workflow.validate_dependencies();
    assert!(validation_result.is_ok(), "Dependencies should be valid");
    
    // 验证执行顺序
    let execution_order = workflow.get_execution_order().unwrap();
    assert_eq!(execution_order.len(), 2, "Should have 2 execution levels");
    assert_eq!(execution_order[0], vec!["data_processor"]);
    assert_eq!(execution_order[1], vec!["analyzer"]);
    
    println!("✅ DAG workflow validation passed!");
    println!("Execution order: {:?}", execution_order);
}

#[test]
fn test_workflow_with_invalid_dependencies() {
    let mut worker_inputs = HashMap::new();
    worker_inputs.insert("data".to_string(), "nonexistent_worker/output".to_string());
    
    let worker = Worker {
        name: "test_worker".to_string(),
        workflow_name: "invalid_test".to_string(),
        command: "echo".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
        inputs: Some(worker_inputs),
        outputs: None,
    };
    
    let workflow = Workflow {
        name: "invalid_test".to_string(),
        working_dir: None,
        workers: vec![worker],
    };
    
    // 验证依赖关系应该失败
    let validation_result = workflow.validate_dependencies();
    assert!(validation_result.is_err(), "Dependencies should be invalid");
    
    println!("✅ Invalid workflow correctly rejected!");
}
