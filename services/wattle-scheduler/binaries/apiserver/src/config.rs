use coordinator::CoordinatorConfig;
use serde::Deserialize;
use std::{fs, path::Path};

#[derive(Debug, Clone, Deserialize, Default)]
pub struct Config {
    pub server: ServerConfig,
    pub coordinator: Option<CoordinatorConfig>,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct ServerConfig {
    pub host: Option<String>,
    pub port: Option<u16>,
}

impl Config {
    pub fn new<P: AsRef<Path>>(config_path: P) -> eyre::Result<Self> {
        let content = fs::read_to_string(config_path.as_ref())?;
        let config: Config = toml::from_str(&content)?;
        Ok(config)
    }

    pub fn get_addr(&self) -> String {
        let host = self.server.host.as_deref().unwrap_or("0.0.0.0");
        let port = self.server.port.unwrap_or(8080);
        format!("{}:{}", host, port)
    }
}
