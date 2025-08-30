use clap::{Parser, Subcommand};
use serde_json::json;
use std::collections::HashMap;

#[derive(Parser)]
#[command(name = "wattle-cli")]
#[command(about = "A CLI for managing Wattle workflows")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// List all workflows
    List,
    /// Create a new workflow from JSON
    Create {
        /// JSON string or file path containing workflow definition
        #[arg(short, long)]
        input: String,
    },
    /// Run a workflow by name
    Run {
        /// Name of the workflow to run
        name: String,
    },
    /// Get workflow status
    Status {
        /// Name of the workflow
        name: String,
    },
    /// Generate example workflow
    Example,
}

fn main() -> eyre::Result<()> {
    let cli = Cli::parse();

    match &cli.command {
        Commands::List => {
            println!("📋 Listing workflows...");
            println!("This would connect to the API server to list workflows");
            println!("Example: GET http://localhost:3000/api/v1/workflows");
        }
        Commands::Create { input } => {
            println!("🔨 Creating workflow...");
            println!("Input: {}", input);
            
            // Try to parse as JSON or read from file
            let workflow_json = if input.starts_with('{') {
                // Direct JSON input
                input.clone()
            } else {
                // File path
                match std::fs::read_to_string(input) {
                    Ok(content) => content,
                    Err(e) => {
                        eprintln!("❌ Failed to read file '{}': {}", input, e);
                        return Ok(());
                    }
                }
            };
            
            // Validate JSON
            match serde_json::from_str::<serde_json::Value>(&workflow_json) {
                Ok(json) => {
                    println!("✅ Valid JSON workflow definition:");
                    println!("{}", serde_json::to_string_pretty(&json)?);
                    println!("This would be sent to: POST http://localhost:3000/api/v1/workflows");
                }
                Err(e) => {
                    eprintln!("❌ Invalid JSON: {}", e);
                }
            }
        }
        Commands::Run { name } => {
            println!("🚀 Running workflow: {}", name);
            println!("This would send: POST http://localhost:3000/api/v1/workflows/{}/run", name);
        }
        Commands::Status { name } => {
            println!("📊 Getting status for workflow: {}", name);
            println!("This would send: GET http://localhost:3000/api/v1/workflows/{}", name);
        }
        Commands::Example => {
            println!("📝 Generating example workflow...");
            generate_example_workflow()?;
        }
    }

    Ok(())
}

fn generate_example_workflow() -> eyre::Result<()> {
    let mut env_vars = HashMap::new();
    env_vars.insert("NODE_ENV".to_string(), "production".to_string());
    env_vars.insert("API_URL".to_string(), "https://api.example.com".to_string());

    let example_workflow = json!({
        "name": "web-deployment-example",
        "working_dir": "/app",
        "workers": [
            {
                "name": "frontend-build",
                "workflow_name": "web-deployment-example",
                "command": "npm run build",
                "args": ["--production"],
                "working_dir": "/app/frontend",
                "env_vars": env_vars
            },
            {
                "name": "backend-build",
                "workflow_name": "web-deployment-example", 
                "command": "cargo build --release",
                "args": null,
                "working_dir": "/app/backend",
                "env_vars": null
            },
            {
                "name": "deploy",
                "workflow_name": "web-deployment-example",
                "command": "kubectl apply -f deployment.yaml",
                "args": null,
                "working_dir": "/app",
                "env_vars": null
            }
        ]
    });

    let pretty_json = serde_json::to_string_pretty(&example_workflow)?;
    println!("{}", pretty_json);
    
    println!("\n💡 You can save this to a file and use it with:");
    println!("   wattle-cli create --input workflow.json");
    
    // Also save to file
    std::fs::write("example_workflow.json", &pretty_json)?;
    println!("\n✅ Example saved to 'example_workflow.json'");

    Ok(())
}
