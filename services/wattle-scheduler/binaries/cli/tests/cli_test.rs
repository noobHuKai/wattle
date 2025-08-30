use std::process::Command;

#[test]
fn test_cli_help() {
    let output = Command::new("cargo")
        .args(&["run", "--bin", "cli", "--", "--help"])
        .output()
        .expect("Failed to execute command");

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("A CLI for managing Wattle workflows"));
    assert!(stdout.contains("list"));
    assert!(stdout.contains("create"));
    assert!(stdout.contains("run"));
    assert!(stdout.contains("status"));
    assert!(stdout.contains("example"));
}

#[test]
fn test_cli_example_command() {
    let output = Command::new("cargo")
        .args(&["run", "--bin", "cli", "--", "example"])
        .output()
        .expect("Failed to execute command");

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("web-deployment-example"));
    assert!(stdout.contains("frontend-build"));
    assert!(stdout.contains("backend-build"));
    assert!(stdout.contains("deploy"));
    assert!(output.status.success());
}

#[test]
fn test_workflow_json_validation() {
    use serde_json::json;
    
    // Test valid workflow JSON
    let valid_workflow = json!({
        "name": "test-workflow",
        "working_dir": "/tmp",
        "workers": [
            {
                "name": "test-worker",
                "workflow_name": "test-workflow",
                "command": "echo test",
                "args": null,
                "working_dir": null,
                "env_vars": null
            }
        ]
    });
    
    let json_str = serde_json::to_string(&valid_workflow).unwrap();
    let parsed: serde_json::Value = serde_json::from_str(&json_str).unwrap();
    
    assert_eq!(parsed["name"], "test-workflow");
    assert_eq!(parsed["workers"].as_array().unwrap().len(), 1);
    assert_eq!(parsed["workers"][0]["name"], "test-worker");
}

#[test]
fn test_cli_create_with_json() {
    let test_json = r#"{"name":"test","working_dir":"/tmp","workers":[]}"#;
    
    let output = Command::new("cargo")
        .args(&["run", "--bin", "cli", "--", "create", "--input", test_json])
        .output()
        .expect("Failed to execute command");

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Valid JSON workflow definition"));
    assert!(stdout.contains("test"));
    assert!(output.status.success());
}

#[test]
fn test_cli_create_with_invalid_json() {
    let invalid_json = r#"{"name":"test","invalid"}"#;
    
    let output = Command::new("cargo")
        .args(&["run", "--bin", "cli", "--", "create", "--input", invalid_json])
        .output()
        .expect("Failed to execute command");

    let stderr = String::from_utf8_lossy(&output.stderr);
    // The command should handle invalid JSON gracefully
    assert!(!output.status.success() || stderr.contains("Invalid JSON"));
}

#[test]
fn test_cli_list_command() {
    let output = Command::new("cargo")
        .args(&["run", "--bin", "cli", "--", "list"])
        .output()
        .expect("Failed to execute command");

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Listing workflows"));
    assert!(stdout.contains("GET http://localhost:3000/api/v1/workflows"));
    assert!(output.status.success());
}

#[test]
fn test_cli_run_command() {
    let output = Command::new("cargo")
        .args(&["run", "--bin", "cli", "--", "run", "test-workflow"])
        .output()
        .expect("Failed to execute command");

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Running workflow: test-workflow"));
    assert!(stdout.contains("POST http://localhost:3000/api/v1/workflows/test-workflow/run"));
    assert!(output.status.success());
}

#[test]
fn test_cli_status_command() {
    let output = Command::new("cargo")
        .args(&["run", "--bin", "cli", "--", "status", "test-workflow"])
        .output()
        .expect("Failed to execute command");

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Getting status for workflow: test-workflow"));
    assert!(stdout.contains("GET http://localhost:3000/api/v1/workflows/test-workflow"));
    assert!(output.status.success());
}
