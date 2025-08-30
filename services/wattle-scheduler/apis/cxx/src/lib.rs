//! Simplified Wattle C++ SDK using cxx bindings

use cxx::CxxString;

#[cxx::bridge]
mod ffi {
    #[derive(Debug)]
    struct WattleResponse {
        success: bool,
        message: String,
        data: String,
    }

    unsafe extern "C++" {
        include!("wattle_cxx.h");
    }

    extern "Rust" {
        type WattleClient;
        fn new_wattle_client(workflow_name: &CxxString, worker_name: &CxxString) -> Result<Box<WattleClient>>;
        fn publish_json(client: &mut WattleClient, topic: &CxxString, data: &CxxString) -> Result<WattleResponse>;
        fn request_json(client: &mut WattleClient, worker: &CxxString, service: &CxxString, data: &CxxString, timeout_ms: u32) -> Result<WattleResponse>;
        fn subscribe_json(client: &mut WattleClient, topic: &CxxString) -> Result<WattleResponse>;
        fn get_workflow_name(client: &WattleClient) -> String;
        fn get_worker_name(client: &WattleClient) -> String;
        fn cleanup_client(client: &mut WattleClient) -> Result<()>;
    }
}

pub struct WattleClient {
    workflow_name: String,
    worker_name: String,
}

pub fn new_wattle_client(workflow_name: &CxxString, worker_name: &CxxString) -> Result<Box<WattleClient>, Box<dyn std::error::Error + Send + Sync>> {
    Ok(Box::new(WattleClient {
        workflow_name: workflow_name.to_string(),
        worker_name: worker_name.to_string(),
    }))
}

pub fn publish_json(client: &mut WattleClient, topic: &CxxString, data: &CxxString) -> Result<ffi::WattleResponse, Box<dyn std::error::Error + Send + Sync>> {
    Ok(ffi::WattleResponse {
        success: true,
        message: format!("Published to {} by {}/{}", topic, client.workflow_name, client.worker_name),
        data: data.to_string(),
    })
}

pub fn request_json(_client: &mut WattleClient, worker: &CxxString, service: &CxxString, data: &CxxString, timeout_ms: u32) -> Result<ffi::WattleResponse, Box<dyn std::error::Error + Send + Sync>> {
    Ok(ffi::WattleResponse {
        success: true,
        message: format!("Request to {}/{} with timeout {}ms", worker, service, timeout_ms),
        data: format!("{{\"status\": \"ok\", \"original_data\": {}}}", data.to_string()),
    })
}

pub fn subscribe_json(client: &mut WattleClient, topic: &CxxString) -> Result<ffi::WattleResponse, Box<dyn std::error::Error + Send + Sync>> {
    Ok(ffi::WattleResponse {
        success: true,
        message: format!("Subscribed to {} by {}/{}", topic, client.workflow_name, client.worker_name),
        data: String::new(),
    })
}

pub fn get_workflow_name(client: &WattleClient) -> String {
    client.workflow_name.clone()
}

pub fn get_worker_name(client: &WattleClient) -> String {
    client.worker_name.clone()
}

pub fn cleanup_client(_client: &mut WattleClient) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    Ok(())
}