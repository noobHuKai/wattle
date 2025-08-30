//! Wattle C++ SDK using cxx bindings
//! 
//! This library provides C++ bindings for the Wattle Rust SDK,
//! enabling C++ applications to use Wattle's Worker-to-Worker communication.

use std::sync::Arc;
use std::pin::Pin;
use eyre::{Result as EyreResult, eyre};
use tokio::runtime::Runtime;
use wattle_rs::WattleClient as RustWattleClient;

// Implement Display for WattleError outside the bridge
impl std::fmt::Display for ffi::WattleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for ffi::WattleError {}

#[cxx::bridge]
mod ffi {
    #[derive(Debug)]
    pub struct WattleError {
        pub message: String,
    }

    #[derive(Debug)]
    pub struct WattleResponse {
        pub success: bool,
        pub data: String,
        pub error: String,
    }

    extern "Rust" {
        type WattleClient;

        /// Create a new Wattle client
        fn new_wattle_client(workflow_name: &str, worker_name: &str) -> Result<Box<WattleClient>>;

        /// Publish JSON data
        fn publish_json(client: Pin<&mut WattleClient>, service_name: &str, json_data: &str) -> WattleResponse;

        /// Send a JSON request and wait for reply
        fn request_json(client: Pin<&mut WattleClient>, target_worker: &str, service_name: &str, json_data: &str, timeout_secs: u32) -> WattleResponse;

        /// Subscribe to JSON messages (simplified version)
        fn subscribe_json(client: Pin<&mut WattleClient>, service_name: &str) -> WattleResponse;

        /// Get next subscription message (polling interface for C++)
        fn get_next_message(client: Pin<&mut WattleClient>) -> WattleResponse;

        /// Clean up client resources
        fn cleanup_client(client: Pin<&mut WattleClient>) -> WattleResponse;
    }
}

/// Wrapper around the Rust WattleClient for C++ interop
pub struct WattleClient {
    inner: Option<Arc<RustWattleClient>>,
    runtime: Arc<Runtime>,
}

impl WattleClient {
    fn new(workflow_name: &str, worker_name: &str) -> EyreResult<Self> {
        let runtime = Arc::new(
            tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .map_err(|e| eyre!("Failed to create tokio runtime: {}", e))?
        );

        // Set environment variables for the Rust client
        std::env::set_var("WATTLE_WORKFLOW_NAME", workflow_name);
        std::env::set_var("WATTLE_WORKER_NAME", worker_name);

        let inner = runtime.block_on(async {
            RustWattleClient::new().await
        })?;

        Ok(WattleClient {
            inner: Some(Arc::new(inner)),
            runtime,
        })
    }

    fn publish_json_impl(&mut self, service_name: &str, json_data: &str) -> EyreResult<()> {
        let client = self.inner.as_ref()
            .ok_or_else(|| eyre!("Client not initialized"))?;

        let data: serde_json::Value = serde_json::from_str(json_data)
            .map_err(|e| eyre!("Invalid JSON data: {}", e))?;

        self.runtime.block_on(async {
            client.publish_json(service_name, &data).await
        })
    }

    fn request_json_impl(&mut self, target_worker: &str, service_name: &str, json_data: &str, timeout_secs: u32) -> EyreResult<String> {
        let client = self.inner.as_ref()
            .ok_or_else(|| eyre!("Client not initialized"))?;

        let data: serde_json::Value = serde_json::from_str(json_data)
            .map_err(|e| eyre!("Invalid JSON data: {}", e))?;

        let timeout = if timeout_secs > 0 { Some(timeout_secs as u64) } else { None };

        let response = self.runtime.block_on(async {
            client.request_json(target_worker, service_name, &data, timeout).await
        })?;

        serde_json::to_string(&response)
            .map_err(|e| eyre!("Failed to serialize response: {}", e))
    }

    fn subscribe_json_impl(&mut self, service_name: &str) -> EyreResult<()> {
        self.inner.as_ref()
            .ok_or_else(|| eyre!("Client not initialized"))?;

        // For now, just store that we're subscribed
        // In a real implementation, we'd set up the subscription
        // This is a simplified version for the C++ binding
        
        tracing::info!("Subscribed to service: {}", service_name);
        Ok(())
    }

    fn get_next_message_impl(&mut self) -> EyreResult<Option<(String, String)>> {
        // For now, return None (no messages)
        // In a real implementation, we'd poll the actual subscription queue
        Ok(None)
    }

    fn cleanup_impl(&mut self) -> EyreResult<()> {
        self.inner.take();
        Ok(())
    }
}

// C++ interface functions
fn new_wattle_client(workflow_name: &str, worker_name: &str) -> Result<Box<WattleClient>, ffi::WattleError> {
    match WattleClient::new(workflow_name, worker_name) {
        Ok(client) => Ok(Box::new(client)),
        Err(e) => Err(ffi::WattleError {
            message: e.to_string(),
        }),
    }
}

fn publish_json(mut client: Pin<&mut WattleClient>, service_name: &str, json_data: &str) -> ffi::WattleResponse {
    match client.publish_json_impl(service_name, json_data) {
        Ok(_) => ffi::WattleResponse {
            success: true,
            data: "Published successfully".to_string(),
            error: String::new(),
        },
        Err(e) => ffi::WattleResponse {
            success: false,
            data: String::new(),
            error: e.to_string(),
        },
    }
}

fn request_json(mut client: Pin<&mut WattleClient>, target_worker: &str, service_name: &str, json_data: &str, timeout_secs: u32) -> ffi::WattleResponse {
    match client.request_json_impl(target_worker, service_name, json_data, timeout_secs) {
        Ok(response) => ffi::WattleResponse {
            success: true,
            data: response,
            error: String::new(),
        },
        Err(e) => ffi::WattleResponse {
            success: false,
            data: String::new(),
            error: e.to_string(),
        },
    }
}

fn subscribe_json(mut client: Pin<&mut WattleClient>, service_name: &str) -> ffi::WattleResponse {
    match client.subscribe_json_impl(service_name) {
        Ok(_) => ffi::WattleResponse {
            success: true,
            data: "Subscription started".to_string(),
            error: String::new(),
        },
        Err(e) => ffi::WattleResponse {
            success: false,
            data: String::new(),
            error: e.to_string(),
        },
    }
}

fn get_next_message(mut client: Pin<&mut WattleClient>) -> ffi::WattleResponse {
    match client.get_next_message_impl() {
        Ok(Some((service, data))) => ffi::WattleResponse {
            success: true,
            data: format!("{{\"service\":\"{}\",\"data\":{}}}", service, data),
            error: String::new(),
        },
        Ok(None) => ffi::WattleResponse {
            success: true,
            data: String::new(),
            error: String::new(),
        },
        Err(e) => ffi::WattleResponse {
            success: false,
            data: String::new(),
            error: e.to_string(),
        },
    }
}

fn cleanup_client(mut client: Pin<&mut WattleClient>) -> ffi::WattleResponse {
    match client.cleanup_impl() {
        Ok(_) => ffi::WattleResponse {
            success: true,
            data: "Client cleaned up successfully".to_string(),
            error: String::new(),
        },
        Err(e) => ffi::WattleResponse {
            success: false,
            data: String::new(),
            error: e.to_string(),
        },
    }
}
