//! Simple Wattle Python SDK

use pyo3::prelude::*;

#[pyclass]
struct WattleClient {
    workflow_name: String,
    worker_name: String,
}

#[pymethods]
impl WattleClient {
    #[new]
    fn new(workflow_name: String, worker_name: String) -> Self {
        WattleClient {
            workflow_name,
            worker_name,
        }
    }

    fn publish_json(&self, topic: &str, data: &str) -> String {
        format!("Published to {}: {} (workflow: {}, worker: {})",
                topic, data, self.workflow_name, self.worker_name)
    }

    fn request_json(&self, worker: &str, service: &str, data: &str, timeout_ms: u32) -> String {
        format!("Request to {}/{}: {} (timeout: {}ms) - Mock response",
                worker, service, data, timeout_ms)
    }

    fn subscribe(&self, topic: &str) -> String {
        format!("Subscribed to {} (workflow: {}, worker: {})",
                topic, self.workflow_name, self.worker_name)
    }

    fn get_workflow_name(&self) -> String {
        self.workflow_name.clone()
    }

    fn get_worker_name(&self) -> String {
        self.worker_name.clone()
    }

    fn close(&self) -> String {
        "Client closed".to_string()
    }
}

#[pyfunction]
fn get_version() -> String {
    "0.1.0".to_string()
}

#[pymodule]
fn wattle_py(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<WattleClient>()?;
    m.add_function(wrap_pyfunction!(get_version, m)?)?;
    Ok(())
}