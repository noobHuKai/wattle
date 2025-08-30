use core::{Worker, Workflow};
use std::collections::HashMap;

fn main() {
    println!("=== Wattle Workflow DAG Example ===");
    
    // 示例: 一个数据处理管道
    // data_fetcher -> [data_processor_a, data_processor_b] -> aggregator -> reporter
    
    // Step 1: 数据获取器 (没有依赖)
    let mut fetcher_outputs = HashMap::new();
    fetcher_outputs.insert("raw_data".to_string(), "/tmp/raw_data.json".to_string());
    fetcher_outputs.insert("metadata".to_string(), "/tmp/metadata.json".to_string());
    
    let data_fetcher = Worker {

        name: "data_fetcher".to_string(),
        workflow_name: "data_pipeline".to_string(),
        command: "fetch_data".to_string(),
        args: Some(vec!["--source".to_string(), "database".to_string()]),
        working_dir: Some("/tmp/work".to_string()),
        env_vars: {
            let mut env = HashMap::new();
            env.insert("DB_URL".to_string(), "postgresql://localhost/data".to_string());
            Some(env)
        },
        inputs: None,
        outputs: Some(fetcher_outputs),
    };
    
    // Step 2a: 数据处理器 A (依赖 data_fetcher)
    let mut processor_a_inputs = HashMap::new();
    processor_a_inputs.insert("input_data".to_string(), "data_fetcher/raw_data".to_string());
    processor_a_inputs.insert("config".to_string(), "data_fetcher/metadata".to_string());
    
    let mut processor_a_outputs = HashMap::new();
    processor_a_outputs.insert("processed_data_a".to_string(), "/tmp/processed_a.json".to_string());
    
    let data_processor_a = Worker {
        name: "data_processor_a".to_string(),
        workflow_name: "data_pipeline".to_string(),
        command: "process_data".to_string(),
        args: Some(vec!["--type".to_string(), "analytics".to_string()]),
        working_dir: Some("/tmp/work".to_string()),
        env_vars: None,
        inputs: Some(processor_a_inputs),
        outputs: Some(processor_a_outputs),
    };
    
    // Step 2b: 数据处理器 B (依赖 data_fetcher, 可与 processor_a 并行)
    let mut processor_b_inputs = HashMap::new();
    processor_b_inputs.insert("input_data".to_string(), "data_fetcher/raw_data".to_string());
    
    let mut processor_b_outputs = HashMap::new();
    processor_b_outputs.insert("processed_data_b".to_string(), "/tmp/processed_b.json".to_string());
    
    let data_processor_b = Worker {
        name: "data_processor_b".to_string(),
        workflow_name: "data_pipeline".to_string(),
        command: "process_data".to_string(),
        args: Some(vec!["--type".to_string(), "statistics".to_string()]),
        working_dir: Some("/tmp/work".to_string()),
        env_vars: None,
        inputs: Some(processor_b_inputs),
        outputs: Some(processor_b_outputs),
    };
    
    // Step 3: 聚合器 (依赖两个处理器)
    let mut aggregator_inputs = HashMap::new();
    aggregator_inputs.insert("analytics_data".to_string(), "data_processor_a/processed_data_a".to_string());
    aggregator_inputs.insert("stats_data".to_string(), "data_processor_b/processed_data_b".to_string());
    
    let mut aggregator_outputs = HashMap::new();
    aggregator_outputs.insert("final_result".to_string(), "/tmp/aggregated.json".to_string());
    
    let aggregator = Worker {
        name: "aggregator".to_string(),
        workflow_name: "data_pipeline".to_string(),
        command: "aggregate_data".to_string(),
        args: Some(vec!["--output".to_string(), "/tmp/aggregated.json".to_string()]),
        working_dir: Some("/tmp/work".to_string()),
        env_vars: None,
        inputs: Some(aggregator_inputs),
        outputs: Some(aggregator_outputs),
    };
    
    // Step 4: 报告生成器 (依赖聚合器)
    let mut reporter_inputs = HashMap::new();
    reporter_inputs.insert("aggregated_data".to_string(), "aggregator/final_result".to_string());
    
    let reporter = Worker {
        name: "reporter".to_string(),
        workflow_name: "data_pipeline".to_string(),
        command: "generate_report".to_string(),
        args: Some(vec!["--template".to_string(), "summary".to_string()]),
        working_dir: Some("/tmp/work".to_string()),
        env_vars: None,
        inputs: Some(reporter_inputs),
        outputs: None,
    };
    
    // 创建工作流
    let workflow = Workflow {
        name: "data_pipeline".to_string(),
        working_dir: Some("/tmp".to_string()),
        workers: vec![data_fetcher, data_processor_a, data_processor_b, aggregator, reporter],
    };
    
    // 验证依赖关系
    println!("🔍 Validating workflow dependencies...");
    match workflow.validate_dependencies() {
        Ok(_) => println!("✅ All dependencies are valid!"),
        Err(e) => {
            println!("❌ Dependency validation failed: {}", e);
            return;
        }
    }
    
    // 获取执行顺序
    println!("\n📋 Computing execution order...");
    match workflow.get_execution_order() {
        Ok(execution_levels) => {
            println!("✅ Execution plan generated!");
            println!("\n🚀 Execution Plan:");
            
            for (level, workers) in execution_levels.iter().enumerate() {
                if workers.len() == 1 {
                    println!("  Level {}: {} (sequential)", level, workers[0]);
                } else {
                    println!("  Level {}: {} (parallel)", level, workers.join(", "));
                }
            }
            
            println!("\n📊 Execution Summary:");
            println!("  - Total levels: {}", execution_levels.len());
            println!("  - Total workers: {}", workflow.workers.len());
            println!("  - Parallel workers in level 1: {}", execution_levels.get(1).map(|w| w.len()).unwrap_or(0));
            
            // 模拟执行时间计算
            let estimated_time = execution_levels.len() * 30; // 假设每层级30秒
            println!("  - Estimated execution time: ~{} seconds", estimated_time);
            
        },
        Err(e) => {
            println!("❌ Failed to compute execution order: {}", e);
        }
    }
    
    println!("\n📝 Workflow Configuration:");
    for worker in &workflow.workers {
        println!("  Worker: {}", worker.name);
        if let Some(ref inputs) = worker.inputs {
            println!("    Inputs:");
            for (key, dependency) in inputs {
                println!("      {} <- {}", key, dependency);
            }
        }
        if let Some(ref outputs) = worker.outputs {
            println!("    Outputs:");
            for (key, path) in outputs {
                println!("      {} -> {}", key, path);
            }
        }
    }
    
    println!("\n✨ Example completed successfully!");
}
