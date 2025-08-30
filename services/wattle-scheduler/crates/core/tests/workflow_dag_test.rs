use core::{Worker, Workflow};
use std::collections::HashMap;

#[test]
fn test_workflow_dependency_validation() {
    // 创建一个有依赖关系的工作流
    let mut worker1_outputs = HashMap::new();
    worker1_outputs.insert("data".to_string(), "/path/to/data".to_string());
    worker1_outputs.insert("config".to_string(), "/path/to/config".to_string());
    
    let worker1 = Worker {
        name: "data_processor".to_string(),
        workflow_name: "test_workflow".to_string(),
        command: "process_data".to_string(),
        args: Some(vec!["--input".to_string(), "raw_data".to_string()]),
        working_dir: Some("/tmp/work1".to_string()),
        env_vars: None,
        inputs: None,
        outputs: Some(worker1_outputs),
    };
    
    let mut worker2_inputs = HashMap::new();
    worker2_inputs.insert("processed_data".to_string(), "data_processor/data".to_string());
    worker2_inputs.insert("config_file".to_string(), "data_processor/config".to_string());
    
    let mut worker2_outputs = HashMap::new();
    worker2_outputs.insert("result".to_string(), "/path/to/result".to_string());
    
    let worker2 = Worker {
        name: "analyzer".to_string(),
        workflow_name: "test_workflow".to_string(),
        command: "analyze".to_string(),
        args: Some(vec!["--data".to_string(), "${processed_data}".to_string()]),
        working_dir: Some("/tmp/work2".to_string()),
        env_vars: None,
        inputs: Some(worker2_inputs),
        outputs: Some(worker2_outputs),
    };
    
    let mut worker3_inputs = HashMap::new();
    worker3_inputs.insert("analysis_result".to_string(), "analyzer/result".to_string());
    
    let worker3 = Worker {
        name: "reporter".to_string(),
        workflow_name: "test_workflow".to_string(),
        command: "generate_report".to_string(),
        args: Some(vec!["--input".to_string(), "${analysis_result}".to_string()]),
        working_dir: Some("/tmp/work3".to_string()),
        env_vars: None,
        inputs: Some(worker3_inputs),
        outputs: None,
    };
    
    let workflow = Workflow {
        name: "test_workflow".to_string(),
        working_dir: Some("/tmp".to_string()),
        workers: vec![worker1, worker2, worker3],
    };
    
    // 测试依赖验证
    let result = workflow.validate_dependencies();
    assert!(result.is_ok(), "Workflow should have valid dependencies");
    
    println!("Workers and their dependencies:");
    for worker in &workflow.workers {
        println!("Worker: {}", worker.name);
        if let Some(ref inputs) = worker.inputs {
            for (input_key, dependency_path) in inputs {
                println!("  Input: {} <- {}", input_key, dependency_path);
            }
        } else {
            println!("  No inputs");
        }
        if let Some(ref outputs) = worker.outputs {
            for (output_key, _) in outputs {
                println!("  Output: {}", output_key);
            }
        } else {
            println!("  No outputs");  
        }
    }
    
    // 测试拓扑排序
    let execution_order = workflow.get_execution_order().unwrap();
    
    println!("Actual execution order: {:?}", execution_order);
    
    // 验证执行顺序
    assert_eq!(execution_order.len(), 3, "Should have 3 execution levels");
    
    // Level 0: data_processor (no dependencies)
    assert_eq!(execution_order[0], vec!["data_processor"]);
    
    // Level 1: analyzer (depends on data_processor)  
    assert_eq!(execution_order[1], vec!["analyzer"]);
    
    // Level 2: reporter (depends on analyzer)
    assert_eq!(execution_order[2], vec!["reporter"]);
    
    println!("Execution order: {:?}", execution_order);
}

#[test]
fn test_workflow_invalid_dependency() {
    // 创建一个有无效依赖的工作流
    let mut worker1_inputs = HashMap::new();
    worker1_inputs.insert("data".to_string(), "nonexistent_worker/data".to_string());
    
    let worker1 = Worker {
        name: "worker1".to_string(),
        workflow_name: "invalid_workflow".to_string(),
        command: "echo".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
        inputs: Some(worker1_inputs),
        outputs: None,
    };
    
    let workflow = Workflow {
        name: "invalid_workflow".to_string(),
        working_dir: None,
        workers: vec![worker1],
    };
    
    // 测试依赖验证应该失败
    let result = workflow.validate_dependencies();
    assert!(result.is_err(), "Workflow should have invalid dependencies");
    
    let error = result.unwrap_err();
    println!("Expected error: {:?}", error);
}

#[test]
fn test_workflow_parallel_execution() {
    // 创建一个可以并行执行的工作流
    let mut worker1_outputs = HashMap::new();
    worker1_outputs.insert("config".to_string(), "/config.json".to_string());
    
    let worker1 = Worker {
        name: "config_generator".to_string(),
        workflow_name: "parallel_workflow".to_string(),
        command: "generate_config".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
        inputs: None,
        outputs: Some(worker1_outputs),
    };
    
    // 两个worker都依赖config_generator，但互不依赖
    let mut worker2_inputs = HashMap::new();
    worker2_inputs.insert("config".to_string(), "config_generator/config".to_string());
    
    let mut worker2_outputs = HashMap::new();
    worker2_outputs.insert("data_a".to_string(), "/data_a.json".to_string());
    
    let worker2 = Worker {
        name: "processor_a".to_string(),
        workflow_name: "parallel_workflow".to_string(),
        command: "process_a".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
        inputs: Some(worker2_inputs),
        outputs: Some(worker2_outputs),
    };
    
    let mut worker3_inputs = HashMap::new();
    worker3_inputs.insert("config".to_string(), "config_generator/config".to_string());
    
    let mut worker3_outputs = HashMap::new();
    worker3_outputs.insert("data_b".to_string(), "/data_b.json".to_string());
    
    let worker3 = Worker {
        name: "processor_b".to_string(),
        workflow_name: "parallel_workflow".to_string(),
        command: "process_b".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
        inputs: Some(worker3_inputs),
        outputs: Some(worker3_outputs),
    };
    
    let workflow = Workflow {
        name: "parallel_workflow".to_string(),
        working_dir: None,
        workers: vec![worker1, worker2, worker3],
    };
    
    // 测试执行顺序
    let execution_order = workflow.get_execution_order().unwrap();
    
    // 应该有两个执行层级
    assert_eq!(execution_order.len(), 2);
    
    // Level 0: config_generator
    assert_eq!(execution_order[0], vec!["config_generator"]);
    
    // Level 1: processor_a 和 processor_b 可以并行执行
    let mut level1 = execution_order[1].clone();
    level1.sort();
    assert_eq!(level1, vec!["processor_a", "processor_b"]);
    
    println!("Parallel execution order: {:?}", execution_order);
}

#[test] 
fn test_workflow_circular_dependency() {
    // 创建循环依赖的工作流
    let mut worker1_inputs = HashMap::new();
    worker1_inputs.insert("data".to_string(), "worker2/output".to_string());
    let mut worker1_outputs = HashMap::new();
    worker1_outputs.insert("output".to_string(), "/output1".to_string());
    
    let worker1 = Worker {
        name: "worker1".to_string(),
        workflow_name: "circular_workflow".to_string(),
        command: "echo".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
        inputs: Some(worker1_inputs),
        outputs: Some(worker1_outputs),
    };
    
    let mut worker2_inputs = HashMap::new();
    worker2_inputs.insert("data".to_string(), "worker1/output".to_string());
    let mut worker2_outputs = HashMap::new();
    worker2_outputs.insert("output".to_string(), "/output2".to_string());
    
    let worker2 = Worker {
        name: "worker2".to_string(),
        workflow_name: "circular_workflow".to_string(),
        command: "echo".to_string(),
        args: None,
        working_dir: None,
        env_vars: None,
        inputs: Some(worker2_inputs),
        outputs: Some(worker2_outputs),
    };
    
    let workflow = Workflow {
        name: "circular_workflow".to_string(),
        working_dir: None,
        workers: vec![worker1, worker2],
    };
    
    // 验证依赖关系
    let validation_result = workflow.validate_dependencies();
    assert!(validation_result.is_ok(), "Dependency validation should pass");
    
    // 拓扑排序应该检测到循环依赖
    let execution_result = workflow.get_execution_order();
    assert!(execution_result.is_err(), "Should detect circular dependency");
    
    println!("Expected circular dependency error: {:?}", execution_result.unwrap_err());
}
