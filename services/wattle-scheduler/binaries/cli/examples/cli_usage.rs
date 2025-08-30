// Example showing how to use the Wattle CLI programmatically
// This would be useful for automation scripts or integration testing

use serde_json::json;
use std::{collections::HashMap, fs};

fn main() -> eyre::Result<()> {
    println!("=== Wattle CLI Usage Examples ===\n");

    // Example 1: Create workflow files
    println!("1. Creating example workflow files...");
    create_example_workflows()?;

    // Example 2: Show CLI usage patterns
    println!("\n2. CLI Usage Patterns:");
    show_cli_patterns();

    // Example 3: Demonstrate JSON validation
    println!("\n3. JSON Validation Examples:");
    demonstrate_json_validation()?;

    println!("\n=== Examples completed! ===");
    println!("Check the generated workflow files and try the CLI commands shown above.");

    Ok(())
}

fn create_example_workflows() -> eyre::Result<()> {
    // Simple workflow
    let simple_workflow = json!({
        "name": "simple-task",
        "working_dir": "/tmp",
        "workers": [
            {
                "name": "hello-world",
                "workflow_name": "simple-task",
                "command": "echo 'Hello from Wattle!'",
                "args": null,
                "working_dir": null,
                "env_vars": null
            }
        ]
    });

    fs::write("simple_workflow.json", serde_json::to_string_pretty(&simple_workflow)?)?;
    println!("  ✅ Created simple_workflow.json");

    // CI/CD pipeline workflow
    let mut ci_env = HashMap::new();
    ci_env.insert("CI".to_string(), "true".to_string());
    ci_env.insert("NODE_ENV".to_string(), "production".to_string());

    let ci_workflow = json!({
        "name": "ci-cd-pipeline",
        "working_dir": "/app",
        "workers": [
            {
                "name": "install-deps",
                "workflow_name": "ci-cd-pipeline",
                "command": "npm install",
                "args": ["--frozen-lockfile"],
                "working_dir": "/app",
                "env_vars": ci_env
            },
            {
                "name": "run-tests",
                "workflow_name": "ci-cd-pipeline",
                "command": "npm test",
                "args": ["--coverage", "--watchAll=false"],
                "working_dir": "/app",
                "env_vars": ci_env
            },
            {
                "name": "build",
                "workflow_name": "ci-cd-pipeline",
                "command": "npm run build",
                "args": null,
                "working_dir": "/app",
                "env_vars": ci_env
            },
            {
                "name": "deploy",
                "workflow_name": "ci-cd-pipeline",
                "command": "docker build -t myapp . && docker push myapp",
                "args": null,
                "working_dir": "/app",
                "env_vars": null
            }
        ]
    });

    fs::write("ci_cd_workflow.json", serde_json::to_string_pretty(&ci_workflow)?)?;
    println!("  ✅ Created ci_cd_workflow.json");

    // Data processing workflow
    let mut data_env = HashMap::new();
    data_env.insert("PYTHONPATH".to_string(), "/app/src".to_string());
    data_env.insert("DATA_SOURCE".to_string(), "postgresql://localhost/data".to_string());

    let data_workflow = json!({
        "name": "data-processing",
        "working_dir": "/app",
        "workers": [
            {
                "name": "extract-data",
                "workflow_name": "data-processing",
                "command": "python extract.py",
                "args": ["--source", "database", "--format", "csv"],
                "working_dir": "/app/scripts",
                "env_vars": data_env
            },
            {
                "name": "transform-data",
                "workflow_name": "data-processing",
                "command": "python transform.py",
                "args": ["--input", "raw_data.csv", "--output", "processed_data.csv"],
                "working_dir": "/app/scripts",
                "env_vars": data_env
            },
            {
                "name": "load-data",
                "workflow_name": "data-processing",
                "command": "python load.py",
                "args": ["--input", "processed_data.csv", "--target", "warehouse"],
                "working_dir": "/app/scripts",
                "env_vars": data_env
            }
        ]
    });

    fs::write("data_processing_workflow.json", serde_json::to_string_pretty(&data_workflow)?)?;
    println!("  ✅ Created data_processing_workflow.json");

    Ok(())
}

fn show_cli_patterns() {
    println!("  Basic Commands:");
    println!("    cargo run --bin cli -- --help");
    println!("    cargo run --bin cli -- list");
    println!("    cargo run --bin cli -- example");
    println!();
    
    println!("  Workflow Management:");
    println!("    cargo run --bin cli -- create --input simple_workflow.json");
    println!("    cargo run --bin cli -- create --input '{{\"name\":\"test\",\"workers\":[]}}'");
    println!("    cargo run --bin cli -- run simple-task");
    println!("    cargo run --bin cli -- status simple-task");
    println!();
    
    println!("  Real-world Usage:");
    println!("    # Create and run a CI/CD pipeline");
    println!("    cargo run --bin cli -- create --input ci_cd_workflow.json");
    println!("    cargo run --bin cli -- run ci-cd-pipeline");
    println!();
    println!("    # Data processing workflow");
    println!("    cargo run --bin cli -- create --input data_processing_workflow.json");
    println!("    cargo run --bin cli -- status data-processing");
}

fn demonstrate_json_validation() -> eyre::Result<()> {
    println!("  Valid JSON examples:");
    
    // Minimal workflow
    let minimal = json!({
        "name": "minimal",
        "workers": []
    });
    println!("    Minimal: {}", minimal);

    // Workflow with environment variables
    let with_env = json!({
        "name": "with-env",
        "working_dir": "/app",
        "workers": [{
            "name": "env-test",
            "workflow_name": "with-env",
            "command": "printenv",
            "env_vars": {"TEST": "value"}
        }]
    });
    println!("    With env: {}", serde_json::to_string(&with_env)?);

    println!("\n  Invalid JSON examples (these would fail):");
    println!("    Missing name: {{\"workers\":[]}}");
    println!("    Invalid syntax: {{\"name\":\"test\",}}"); // trailing comma
    println!("    Wrong types: {{\"name\":123,\"workers\":\"not_array\"}}");

    Ok(())
}
