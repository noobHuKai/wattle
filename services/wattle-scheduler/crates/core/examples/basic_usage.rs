use core::{Worker, Workflow, WorkerStatus};
use std::collections::HashMap;

fn main() {
    println!("=== Wattle Core Types Example ===\n");

    // Create a worker with environment variables
    let mut env_vars = HashMap::new();
    env_vars.insert("NODE_ENV".to_string(), "production".to_string());
    env_vars.insert("PORT".to_string(), "3000".to_string());

    let web_worker = Worker {
        name: "web-server".to_string(),
        workflow_name: "web-app-deployment".to_string(),
        command: "node server.js".to_string(),
        args: Some(vec!["--port".to_string(), "3000".to_string()]),
        working_dir: Some("/app".to_string()),
        env_vars: Some(env_vars),
    };

    println!("Created web worker:");
    println!("  Name: {}", web_worker.name);
    println!("  Command: {}", web_worker.command);
    println!("  Working Dir: {:?}", web_worker.working_dir);
    println!("  Args: {:?}", web_worker.args);
    println!();

    // Create a database worker
    let db_worker = Worker {
        name: "database-migration".to_string(),
        workflow_name: "web-app-deployment".to_string(),
        command: "npm run migrate".to_string(),
        args: None,
        working_dir: Some("/app".to_string()),
        env_vars: None,
    };

    // Create a workflow with multiple workers
    let workflow = Workflow {
        name: "web-app-deployment".to_string(),
        working_dir: Some("/app".to_string()),
        workers: vec![db_worker, web_worker],
    };

    println!("Created workflow: {}", workflow.name);
    println!("Workers in workflow:");
    for (i, worker) in workflow.workers.iter().enumerate() {
        println!("  {}. {} - {}", i + 1, worker.name, worker.command);
    }
    println!();

    // Demonstrate worker status transitions
    println!("Worker Status Examples:");
    let statuses = vec![
        WorkerStatus::Created,
        WorkerStatus::Running,
        WorkerStatus::Completed,
        WorkerStatus::Failed,
        WorkerStatus::Cancelled,
    ];

    for status in statuses {
        println!("  Status: {} (Debug: {:?})", status, status);
    }
    println!();

    // Demonstrate serialization
    println!("Serialization Example:");
    let json_str = serde_json::to_string_pretty(&workflow).unwrap();
    println!("{}", json_str);
}
