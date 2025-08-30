use storage::{init_database, Repositories};
use core::{Workflow, Worker, WorkerStatus};
use std::collections::HashMap;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("=== Wattle Storage Example ===\n");

    // Create a temporary directory for the database
    let temp_dir = tempfile::tempdir()?;
    let db_path = temp_dir.path().join("example.db");
    
    println!("Creating database at: {:?}", db_path);
    
    // Example 1: Initialize database
    println!("\n1. Initializing database...");
    let db = init_database(Some(db_path.to_string_lossy().to_string())).await?;
    println!("✅ Database initialized successfully");

    // Example 2: Create repository instance
    println!("\n2. Creating repositories...");
    let repos = Repositories::new(db_path.to_string_lossy().to_string()).await?;
    println!("✅ Repository instance created");

    // Example 3: Create and store workflows
    println!("\n3. Creating workflows...");
    
    // Simple workflow
    let simple_workflow = Workflow {
        name: "simple-demo".to_string(),
        working_dir: Some("/tmp".to_string()),
        workers: vec![],
    };
    
    repos.insert_workflow(&simple_workflow).await?;
    println!("✅ Created simple workflow: {}", simple_workflow.name);

    // Complex workflow with workers
    let mut env_vars = HashMap::new();
    env_vars.insert("BUILD_ENV".to_string(), "production".to_string());
    env_vars.insert("VERSION".to_string(), "2.0.0".to_string());

    let build_worker = Worker {
        name: "build-service".to_string(),
        workflow_name: "ci-cd-pipeline".to_string(),
        command: "npm run build".to_string(),
        args: Some(vec!["--production".to_string()]),
        working_dir: Some("/app".to_string()),
        env_vars: Some(env_vars.clone()),
    };

    let test_worker = Worker {
        name: "run-tests".to_string(),
        workflow_name: "ci-cd-pipeline".to_string(),
        command: "npm test".to_string(),
        args: Some(vec!["--coverage".to_string()]),
        working_dir: Some("/app".to_string()),
        env_vars: Some(env_vars.clone()),
    };

    let deploy_worker = Worker {
        name: "deploy-service".to_string(),
        workflow_name: "ci-cd-pipeline".to_string(),
        command: "kubectl apply -f deployment.yaml".to_string(),
        args: None,
        working_dir: Some("/app".to_string()),
        env_vars: Some(env_vars),
    };

    let complex_workflow = Workflow {
        name: "ci-cd-pipeline".to_string(),
        working_dir: Some("/app".to_string()),
        workers: vec![build_worker, test_worker, deploy_worker],
    };

    repos.insert_workflow(&complex_workflow).await?;
    println!("✅ Created complex workflow: {}", complex_workflow.name);

    // Insert workers
    repos.insert_workers(&complex_workflow.workers).await?;
    println!("✅ Added {} workers to the workflow", complex_workflow.workers.len());
    for worker in &complex_workflow.workers {
        println!("  - {} - {}", worker.name, worker.command);
    }

    // Example 4: Query workflows
    println!("\n4. Querying workflows...");
    
    let all_workflows = repos.get_all_workflows().await?;
    println!("Found {} workflows:", all_workflows.len());
    for workflow in &all_workflows {
        println!("  - {} (status: {}, created: {})", 
                 workflow.name, workflow.status, workflow.created_at);
    }

    // Get specific workflow
    if let Some(workflow) = repos.get_workflow("ci-cd-pipeline").await? {
        println!("\nWorkflow details for '{}':", workflow.name);
        println!("  Working dir: {:?}", workflow.working_dir);
        println!("  Status: {}", workflow.status);
        println!("  Created: {}", workflow.created_at);
    }

    // Example 5: Query workers
    println!("\n5. Querying workers...");
    
    let workers = repos.get_workers_by_workflow("ci-cd-pipeline").await?;
    println!("Workers in 'ci-cd-pipeline':");
    for worker in &workers {
        println!("  - {}: {}", worker.name, worker.command);
        println!("    Status: {} (created: {})", worker.status, worker.created_at);
        if let Some(ref args) = worker.args {
            println!("    Args: {:?}", args);
        }
        if let Some(ref env) = worker.env_vars {
            println!("    Env vars: {:?}", env);
        }
        println!();
    }

    // Example 6: Worker status management
    println!("6. Managing worker status...");
    
    // Update worker status through different states
    let worker_name = "build-service";
    let workflow_name = "ci-cd-pipeline";
    
    let statuses = vec![
        (WorkerStatus::Running, "Worker started"),
        (WorkerStatus::Completed, "Worker completed successfully"),
    ];
    
    for (status, description) in statuses {
        repos.update_worker_status(workflow_name, worker_name, status.clone()).await?;
        println!("  ✅ {}: {} -> {}", description, worker_name, status);
        
        // Verify the update by checking worker entity
        let workers = repos.get_workers_by_workflow(workflow_name).await?;
        if let Some(worker) = workers.iter().find(|w| w.name == worker_name) {
            println!("    Confirmed: {} status is now {}", worker.name, worker.status);
        }
    }

    // Example 7: Check existence
    println!("\n7. Checking existence...");
    
    let exists = repos.workflow_exists("ci-cd-pipeline").await?;
    println!("Workflow 'ci-cd-pipeline' exists: {}", exists);
    
    let exists = repos.workflow_exists("non-existent").await?;
    println!("Workflow 'non-existent' exists: {}", exists);
    
    let worker_exists = repos.worker_exists("ci-cd-pipeline", "build-service").await?;
    println!("Worker 'build-service' in 'ci-cd-pipeline' exists: {}", worker_exists);

    // Example 8: Database statistics
    println!("\n8. Database statistics...");
    
    let all_workflows = repos.get_all_workflows().await?;
    let total_workers: usize = {
        let mut count = 0;
        for workflow in &all_workflows {
            let workers = repos.get_workers_by_workflow(&workflow.name).await?;
            count += workers.len();
        }
        count
    };
    
    println!("Total workflows: {}", all_workflows.len());
    println!("Total workers: {}", total_workers);
    
    // Count workers by status
    let mut status_counts: HashMap<WorkerStatus, usize> = HashMap::new();
    for workflow in &all_workflows {
        let workers = repos.get_workers_by_workflow(&workflow.name).await?;
        for worker in workers {
            *status_counts.entry(worker.status).or_insert(0) += 1;
        }
    }
    
    println!("Workers by status:");
    for (status, count) in status_counts {
        println!("  {}: {}", status, count);
    }

    println!("\n=== Example completed! ===");
    println!("Database file: {:?}", db_path);
    println!("You can inspect the database using SQLite tools");

    Ok(())
}
