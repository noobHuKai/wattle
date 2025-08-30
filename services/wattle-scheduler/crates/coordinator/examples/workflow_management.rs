use coordinator::{Coordinator, CoordinatorConfig, Workflow, Worker};
use core::ExecutionConfig;
use std::{collections::HashMap, time::Duration};
use tokio::time::sleep;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("=== Wattle Coordinator Example ===\n");

    // Create a temporary directory for database and logs
    let temp_dir = tempfile::tempdir()?;
    
    // Configure the coordinator
    let config = CoordinatorConfig {
        db_url: Some(temp_dir.path().join("example.db").to_string_lossy().to_string()),
        execution: Some(ExecutionConfig {
            log_dir: Some(temp_dir.path().join("logs").to_path_buf()),
            timeout: Some(Duration::from_secs(30)),
        }),
    };
    
    let coordinator = Coordinator::new(config).await?;
    println!("✅ Created Coordinator with database: {:?}", temp_dir.path().join("example.db"));
    println!();

    // Example 1: Create a simple workflow
    println!("1. Creating a simple workflow...");
    let simple_worker = Worker {
        name: "greeting-worker".to_string(),
        workflow_name: "greeting-workflow".to_string(),
        command: "echo 'Hello from Wattle Coordinator!'".to_string(),
        args: None,
        working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
        env_vars: None,
    };

    let simple_workflow = Workflow {
        name: "greeting-workflow".to_string(),
        working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
        workers: vec![simple_worker],
    };

    coordinator.create_workflow(simple_workflow).await?;
    println!("✅ Created workflow: greeting-workflow");

    // Example 2: Create a complex workflow with multiple workers
    println!("\n2. Creating a complex workflow with multiple workers...");
    
    let mut env_vars = HashMap::new();
    env_vars.insert("PROJECT_NAME".to_string(), "wattle-demo".to_string());
    env_vars.insert("VERSION".to_string(), "1.0.0".to_string());

    let setup_worker = Worker {
        name: "setup-worker".to_string(),
        workflow_name: "build-workflow".to_string(),
        command: "echo 'Setting up project: $PROJECT_NAME v$VERSION'".to_string(),
        args: None,
        working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
        env_vars: Some(env_vars.clone()),
    };

    let build_worker = Worker {
        name: "build-worker".to_string(),
        workflow_name: "build-workflow".to_string(),
        command: "echo 'Building project...' && sleep 1 && echo 'Build completed!'".to_string(),
        args: None,
        working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
        env_vars: Some(env_vars.clone()),
    };

    let test_worker = Worker {
        name: "test-worker".to_string(),
        workflow_name: "build-workflow".to_string(),
        command: "echo 'Running tests...' && sleep 1 && echo 'All tests passed!'".to_string(),
        args: None,
        working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
        env_vars: Some(env_vars),
    };

    let build_workflow = Workflow {
        name: "build-workflow".to_string(),
        working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
        workers: vec![setup_worker, build_worker, test_worker],
    };

    coordinator.create_workflow(build_workflow).await?;
    println!("✅ Created workflow: build-workflow with 3 workers");

    // Example 3: List all workflows
    println!("\n3. Listing all workflows...");
    let workflows = coordinator.list_workflows().await?;
    println!("Found {} workflows:", workflows.len());
    for workflow in &workflows {
        println!("  - {} (created at: {})", workflow.name, workflow.created_at);
    }

    // Example 4: Check if workflows exist
    println!("\n4. Checking workflow existence...");
    let exists = coordinator.workflow_exists("greeting-workflow").await?;
    println!("greeting-workflow exists: {}", exists);
    
    let exists = coordinator.workflow_exists("non-existent").await?;
    println!("non-existent workflow exists: {}", exists);

    // Example 5: Get workflow details
    println!("\n5. Getting workflow details...");
    if let Some(workflow_info) = coordinator.get_workflow_info("build-workflow").await? {
        println!("Workflow: {}", workflow_info.name);
        println!("  Created: {}", workflow_info.created_at);
        println!("  Working Dir: {:?}", workflow_info.working_dir);
    }

    // Example 6: List workers in a workflow
    println!("\n6. Listing workers in build-workflow...");
    let workers = coordinator.list_workers("build-workflow").await?;
    println!("Found {} workers:", workers.len());
    for worker in &workers {
        println!("  - {}: {}", worker.name, worker.command);
        println!("    Status: {} (created at: {})", worker.status, worker.created_at);
    }

    // Example 7: Run workflows
    println!("\n7. Running workflows...");
    
    println!("Running greeting-workflow...");
    coordinator.run_workflow(&"greeting-workflow".to_string()).await?;
    sleep(Duration::from_millis(500)).await;

    println!("Running build-workflow...");
    coordinator.run_workflow(&"build-workflow".to_string()).await?;
    sleep(Duration::from_secs(3)).await; // Give time for all workers to complete

    // Example 8: Worker operations
    println!("\n8. Individual worker operations...");
    
    // Check if a worker exists
    let worker_exists = coordinator.worker_exists("build-workflow", "setup-worker").await?;
    println!("Worker 'setup-worker' exists in 'build-workflow': {}", worker_exists);

    // Get worker info
    if let Ok(worker_info) = coordinator.get_worker_info("build-workflow", "setup-worker").await {
        println!("Worker info for setup-worker:");
        println!("  Command: {}", worker_info.command);
        println!("  Status: {}", worker_info.status);
    }

    println!("\n=== Example completed! ===");
    println!("Database file: {:?}", temp_dir.path().join("example.db"));
    println!("Log directory: {:?}", temp_dir.path().join("logs"));
    
    // Keep temp directory for inspection
    sleep(Duration::from_secs(1)).await;

    Ok(())
}
