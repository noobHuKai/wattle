use coordinator::{Coordinator, CoordinatorConfig, Workflow, Worker};
use core::ExecutionConfig;
use std::{collections::HashMap, time::Duration};
use tokio::time::sleep;

// Simple example showing how the API server components work together
#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("=== API Server Components Example ===\n");

    // Create a temporary directory for the database
    let temp_dir = tempfile::tempdir()?;
    let db_path = temp_dir.path().join("api_example.db");
    
    // Configure coordinator (this is what the API server uses internally)
    let config = CoordinatorConfig {
        db_url: Some(db_path.to_string_lossy().to_string()),
        execution: Some(ExecutionConfig {
            log_dir: Some(temp_dir.path().join("logs").to_path_buf()),
            timeout_secs: Some(30),
            max_parallel_tasks: Some(4),
        }),
    };
    
    println!("Creating coordinator with database: {:?}", db_path);
    let exec_config = ExecutionConfig {
        log_dir: None,
        timeout_secs: Some(30),
        max_parallel_tasks: Some(4),
    };
    let db_config = core::DatabaseConfig {
        max_connections: Some(10),
        connection_timeout_secs: Some(30),
        idle_timeout_secs: Some(300),
    };
    let coordinator = Coordinator::new(config, exec_config, db_config).await?;
    println!("✅ Coordinator created successfully!\n");

    // Demonstrate API operations that the server would handle
    println!("=== Simulating API Operations ===\n");

    // 1. Create workflows (POST /api/v1/workflows)
    println!("1. Creating workflows...");
    
    let web_worker = Worker {

        name: "frontend-build".to_string(),
        workflow_name: "web-deployment".to_string(),
        command: "npm run build".to_string(),
        args: Some(vec!["--production".to_string()]),
        working_dir: Some("/app/frontend".to_string()),
        env_vars: {
            let mut env = HashMap::new();
            env.insert("NODE_ENV".to_string(), "production".to_string());
            env.insert("API_URL".to_string(), "https://api.example.com".to_string());
            Some(env),
        inputs: None,
        outputs: None,
    },
    };

    let api_worker = Worker {

        name: "api-deploy".to_string(),
        workflow_name: "web-deployment".to_string(),
        command: "kubectl apply -f deployment.yaml".to_string(),
        args: None,
        working_dir: Some("/app/api".to_string()),
        env_vars: None,
        inputs: None,
        outputs: None,
    };

    let workflow = Workflow {
        name: "web-deployment".to_string(),
        working_dir: Some("/app".to_string()),
        workers: vec![web_worker, api_worker],
    };

    coordinator.create_workflow(workflow).await?;
    println!("  ✅ Created workflow: web-deployment");

    // Create another workflow
    let test_worker = Worker {

        name: "unit-tests".to_string(),
        workflow_name: "testing-pipeline".to_string(),
        command: "npm test".to_string(),
        args: Some(vec!["--coverage".to_string()]),
        working_dir: Some("/app".to_string()),
        env_vars: None,
        inputs: None,
        outputs: None,
    };

    let test_workflow = Workflow {
        name: "testing-pipeline".to_string(),
        working_dir: Some("/app".to_string()),
        workers: vec![test_worker],
    };

    coordinator.create_workflow(test_workflow).await?;
    println!("  ✅ Created workflow: testing-pipeline");

    // 2. List workflows (GET /api/v1/workflows)
    println!("\n2. Listing workflows...");
    let workflows = coordinator.list_workflows().await?;
    println!("  Found {} workflows:", workflows.len());
    for workflow in &workflows {
        println!("    - {} (created: {})", workflow.name, workflow.created_at);
    }

    // 3. Get specific workflow (GET /api/v1/workflows/{name})
    println!("\n3. Getting specific workflow details...");
    if let Some(workflow_info) = coordinator.get_workflow_info("web-deployment").await? {
        println!("  Workflow: {}", workflow_info.name);
        println!("    Status: {}", workflow_info.status);
        println!("    Working Dir: {:?}", workflow_info.working_dir);
        println!("    Created: {}", workflow_info.created_at);
    }

    // 4. List workers in a workflow (GET /api/v1/workflows/{name}/workers)
    println!("\n4. Listing workers in workflow...");
    let workers = coordinator.list_workers("web-deployment").await?;
    println!("  Workers in 'web-deployment':");
    for worker in &workers {
        println!("    - {}: {}", worker.name, worker.command);
        println!("      Status: {}", WorkerStatus::Created);
    }

    // 5. Run workflow (POST /api/v1/workflows/{name}/run)
    println!("\n5. Running workflow...");
    println!("  Starting 'testing-pipeline'...");
    coordinator.run_workflow(&"testing-pipeline".to_string()).await?;
    println!("  ✅ Workflow execution started");

    // Give some time for the workflow to start
    sleep(Duration::from_millis(500)).await;

    // 6. Check workflow status
    println!("\n6. Checking workflow status...");
    let updated_workflows = coordinator.list_workflows().await?;
    for workflow in updated_workflows {
        println!("  Workflow: {} - Status: {}", workflow.name, workflow.status);
    }

    // 7. Worker operations (GET /api/v1/workflows/{workflow}/workers/{worker})
    println!("\n7. Worker-specific operations...");
    let worker_info = coordinator.get_worker_info("web-deployment", "frontend-build").await?;
    println!("  Worker 'frontend-build' info:");
    println!("    Command: {}", worker_info.command);
    println!("    Status: {}", worker_info.status);
    println!("    Args: {:?}", worker_info.args);

    // 8. Health check simulation
    println!("\n8. Health check simulation...");
    let health_status = serde_json::json!({
        "status": "healthy",
        "workflows": workflows.len(),
        "database": "connected",
        "timestamp": std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs()
    });
    println!("  Health status: {}", health_status);

    println!("\n=== Example completed! ===");
    println!("This demonstrates the core operations that the API server provides:");
    println!("- Workflow management (CRUD operations)");
    println!("- Worker management and monitoring");
    println!("- Workflow execution");
    println!("- Status reporting");
    println!("\nIn a real API server, these operations would be exposed as HTTP endpoints.");
    
    Ok(())
}
