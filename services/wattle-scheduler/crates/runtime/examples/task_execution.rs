use runtime::TaskExecutor;
use core::{ExecutionConfig, Worker};
use std::{collections::HashMap, time::Duration};
use tokio::time::sleep;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("=== Wattle Runtime TaskExecutor Example ===\n");

    // Create a temporary directory for logs
    let temp_dir = tempfile::tempdir()?;
    
    // Configure the executor
    let config = ExecutionConfig {
        log_dir: Some(temp_dir.path().to_path_buf()),
        timeout: Some(Duration::from_secs(30)),
    };
    
    let executor = TaskExecutor::new(config);
    
    println!("Created TaskExecutor with log directory: {:?}", temp_dir.path());
    println!();

    // Example 1: Simple echo command
    println!("1. Running simple echo command...");
    let echo_worker = Worker {
        name: "echo-worker".to_string(),
        workflow_name: "demo-workflow".to_string(),
        command: "echo 'Hello from Wattle!'".to_string(),
        args: None,
        working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
        env_vars: None,
    };

    let result = executor.execute(
        echo_worker,
        async {
            println!("  ✅ Started: echo worker");
        },
        async {
            println!("  ✅ Completed successfully: echo worker");
        },
        async {
            println!("  ❌ Failed: echo worker");
        },
    ).await;

    if let Err(e) = result {
        println!("  Error: {}", e);
    }

    // Wait a bit before next example
    sleep(Duration::from_millis(500)).await;
    println!();

    // Example 2: Command with environment variables
    println!("2. Running command with environment variables...");
    let mut env_vars = HashMap::new();
    env_vars.insert("GREETING".to_string(), "Hello from Wattle Runtime!".to_string());
    env_vars.insert("VERSION".to_string(), "1.0.0".to_string());

    let env_worker = Worker {
        name: "env-worker".to_string(),
        workflow_name: "demo-workflow".to_string(),
        command: r#"sh -c 'echo "Greeting: $GREETING"; echo "Version: $VERSION"'"#.to_string(),
        args: None,
        working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
        env_vars: Some(env_vars),
    };

    let result = executor.execute(
        env_worker,
        async {
            println!("  ✅ Started: env worker");
        },
        async {
            println!("  ✅ Completed successfully: env worker");
        },
        async {
            println!("  ❌ Failed: env worker");
        },
    ).await;

    if let Err(e) = result {
        println!("  Error: {}", e);
    }

    sleep(Duration::from_millis(500)).await;
    println!();

    // Example 3: Multiple commands in sequence
    println!("3. Running multiple commands in sequence...");
    
    let commands = vec![
        ("date-worker", "date", "Display current date"),
        ("uptime-worker", "uptime", "Display system uptime"),
        ("whoami-worker", "whoami", "Display current user"),
    ];

    for (name, cmd, description) in commands {
        println!("  Running: {} - {}", name, description);
        
        let worker = Worker {
            name: name.to_string(),
            workflow_name: "sequence-workflow".to_string(),
            command: cmd.to_string(),
            args: None,
            working_dir: Some(temp_dir.path().to_string_lossy().to_string()),
            env_vars: None,
        };

        let result = executor.execute(
            worker,
            async {
                println!("    Started: {}", name);
            },
            async {
                println!("    Completed: {}", name);
            },
            async {
                println!("    Failed: {}", name);
            },
        ).await;

        if let Err(e) = result {
            println!("    Error: {}", e);
        }

        sleep(Duration::from_millis(200)).await;
    }

    println!();

    // Example 4: List running processes
    println!("4. Checking running processes...");
    let running_processes = executor.list_running_processes().await;
    if running_processes.is_empty() {
        println!("  No processes currently running");
    } else {
        println!("  Running processes:");
        for (pid, worker_name, workflow_name) in running_processes {
            println!("    PID: {}, Worker: {}, Workflow: {}", pid, worker_name, workflow_name);
        }
    }

    println!("\n=== Example completed! ===");
    println!("Check log files in: {:?}", temp_dir.path());

    // Keep temp directory for a moment to inspect logs
    sleep(Duration::from_secs(1)).await;

    Ok(())
}
