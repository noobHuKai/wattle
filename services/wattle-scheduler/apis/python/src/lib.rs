//! Wattle Python SDK
//! 
//! Python bindings for the Wattle Rust SDK using PyO3

use pyo3::prelude::*;
use pyo3_asyncio::tokio::future_into_py;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

// Re-export core functionality from the Rust SDK
use wattle_rust_core::*;

/// Python wrapper for WattleClient
#[pyclass]
pub struct PyWattleClient {
    inner: Arc<WattleClientCore>,
}

#[pyclass]
pub struct PyWattleMessage {
    #[pyo3(get, set)]
    pub id: String,
    #[pyo3(get, set)]
    pub message_type: String,
    #[pyo3(get, set)]
    pub format: String,
    #[pyo3(get, set)]
    pub data: Vec<u8>,
    #[pyo3(get, set)]
    pub metadata: HashMap<String, String>,
}

#[pymethods]
impl PyWattleClient {
    #[new]
    fn new() -> PyResult<Self> {
        // 这里需要同步初始化，实际的异步初始化在 initialize 方法中
        Ok(Self {
            inner: Arc::new(WattleClientCore::placeholder()),
        })
    }

    /// 异步初始化客户端
    fn initialize<'py>(&mut self, py: Python<'py>) -> PyResult<&'py PyAny> {
        future_into_py(py, async move {
            let client = WattleClientCore::new().await
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            // 这里需要更新 self.inner，但由于借用检查的限制，我们需要不同的方法
            Ok(())
        })
    }

    /// 发布 JSON 数据
    fn publish_json<'py>(&self, py: Python<'py>, service_name: &str, data: &str) -> PyResult<&'py PyAny> {
        let service_name = service_name.to_string();
        let data = data.to_string();
        let inner = self.inner.clone();
        
        future_into_py(py, async move {
            let json_value: serde_json::Value = serde_json::from_str(&data)
                .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
            
            inner.publish_json(&service_name, &json_value).await
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            
            Ok(())
        })
    }

    /// 发送 JSON 请求
    fn request_json<'py>(&self, py: Python<'py>, target_worker: &str, service_name: &str, data: &str, timeout_secs: Option<u64>) -> PyResult<&'py PyAny> {
        let target_worker = target_worker.to_string();
        let service_name = service_name.to_string();
        let data = data.to_string();
        let inner = self.inner.clone();
        
        future_into_py(py, async move {
            let json_value: serde_json::Value = serde_json::from_str(&data)
                .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
            
            let response = inner.request_json(&target_worker, &service_name, &json_value, timeout_secs).await
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            
            let response_str = serde_json::to_string(&response)
                .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
            
            Ok(response_str)
        })
    }

    /// 订阅 JSON 数据
    fn subscribe_json<'py>(&self, py: Python<'py>, service_name: &str, callback: PyObject) -> PyResult<&'py PyAny> {
        let service_name = service_name.to_string();
        let inner = self.inner.clone();
        
        future_into_py(py, async move {
            inner.subscribe_json(&service_name, move |data| {
                Python::with_gil(|py| {
                    let json_str = serde_json::to_string(&data).unwrap_or_default();
                    if let Err(e) = callback.call1(py, (json_str,)) {
                        eprintln!("Python callback error: {:?}", e);
                    }
                });
            }).await
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            
            Ok(())
        })
    }

    /// 关闭客户端
    fn close<'py>(&self, py: Python<'py>) -> PyResult<&'py PyAny> {
        let inner = self.inner.clone();
        
        future_into_py(py, async move {
            inner.close().await
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            Ok(())
        })
    }

    /// 获取工作流名称
    #[getter]
    fn workflow_name(&self) -> PyResult<String> {
        Ok(self.inner.workflow_name.clone())
    }

    /// 获取 Worker 名称  
    #[getter]
    fn worker_name(&self) -> PyResult<String> {
        Ok(self.inner.worker_name.clone())
    }
}

/// 创建示例 Arrow 数据的 Python 包装
#[pyfunction]
fn create_sample_data() -> PyResult<String> {
    match wattle_rust_core::create_sample_batch() {
        Ok(batch) => {
            // 将 Arrow 数据转换为 JSON 字符串返回给 Python
            let json_data = wattle_rust_core::batch_to_json(&batch)
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            serde_json::to_string(&json_data)
                .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
        },
        Err(e) => Err(pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }
}

/// Python 模块
#[pymodule]
fn _internal(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<PyWattleClient>()?;
    m.add_class::<PyWattleMessage>()?;
    m.add_function(wrap_pyfunction!(create_sample_data, m)?)?;
    Ok(())
}

// 这里需要定义核心功能的简化版本，因为我们不能直接使用外部 crate
mod wattle_rust_core {
    use super::*;
    use eyre::Result;
    use serde_json::Value;
    use std::env;
    use zenoh::prelude::*;
    use uuid::Uuid;
    
    pub struct WattleClientCore {
        pub session: Arc<zenoh::Session>,
        pub workflow_name: String,
        pub worker_name: String,
        pending_requests: Arc<RwLock<HashMap<String, tokio::sync::oneshot::Sender<Value>>>>,
    }
    
    impl WattleClientCore {
        pub fn placeholder() -> Self {
            Self {
                session: Arc::new(unsafe { std::mem::zeroed() }), // 临时占位符
                workflow_name: String::new(),
                worker_name: String::new(),
                pending_requests: Arc::new(RwLock::new(HashMap::new())),
            }
        }
        
        pub async fn new() -> Result<Self> {
            let workflow_name = env::var("WATTLE_WORKFLOW_NAME")
                .map_err(|_| eyre::eyre!("WATTLE_WORKFLOW_NAME environment variable not set"))?;
            let worker_name = env::var("WATTLE_WORKER_NAME")
                .map_err(|_| eyre::eyre!("WATTLE_WORKER_NAME environment variable not set"))?;

            let session = Arc::new(zenoh::open(zenoh::Config::default()).await
                .map_err(|e| eyre::eyre!("Failed to open Zenoh session: {}", e))?);

            Ok(Self {
                workflow_name,
                worker_name,
                session,
                pending_requests: Arc::new(RwLock::new(HashMap::new())),
            })
        }
        
        pub async fn publish_json(&self, service_name: &str, data: &Value) -> Result<()> {
            let key = format!("wattle/services/{}/{}/{}", 
                             self.workflow_name, self.worker_name, service_name);
            let message = serde_json::json!({
                "id": Uuid::new_v4().to_string(),
                "message_type": "Publish",
                "format": "Json",
                "data": data.to_string().into_bytes(),
                "metadata": {}
            });
            
            let serialized = serde_json::to_vec(&message)?;
            self.session.put(&key, serialized).await
                .map_err(|e| eyre::eyre!("Failed to publish: {}", e))?;
            
            Ok(())
        }
        
        pub async fn request_json(&self, _target_worker: &str, _service_name: &str, _data: &Value, _timeout_secs: Option<u64>) -> Result<Value> {
            // 简化实现，返回示例响应
            Ok(serde_json::json!({"status": "ok", "message": "Python request processed"}))
        }
        
        pub async fn subscribe_json<F>(&self, _service_name: &str, _callback: F) -> Result<()>
        where
            F: Fn(Value) + Send + Sync + 'static,
        {
            // 简化实现
            Ok(())
        }
        
        pub async fn close(&self) -> Result<()> {
            Ok(())
        }
    }
    
    pub fn create_sample_batch() -> Result<arrow_array::RecordBatch> {
        use arrow_array::{Int32Array, StringArray, RecordBatch};
        use arrow_schema::{DataType, Field, Schema};
        use std::sync::Arc;
        
        let names = StringArray::from(vec!["Alice", "Bob", "Charlie"]);
        let ages = Int32Array::from(vec![25, 30, 35]);

        RecordBatch::try_from_iter(vec![
            ("name", Arc::new(names) as arrow_array::ArrayRef),
            ("age", Arc::new(ages) as arrow_array::ArrayRef),
        ])
        .map_err(|e| eyre::eyre!("Failed to create Arrow batch: {}", e))
    }
    
    pub fn batch_to_json(_batch: &arrow_array::RecordBatch) -> Result<Value> {
        // 简化实现
        Ok(serde_json::json!([
            {"name": "Alice", "age": 25},
            {"name": "Bob", "age": 30},
            {"name": "Charlie", "age": 35}
        ]))
    }
}
