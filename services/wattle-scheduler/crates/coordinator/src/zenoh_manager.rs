use eyre::{Result, eyre};
use rand::Rng as _;
use std::process::Command;
use tokio::net::TcpListener;
use tracing::{error, info};

/// Zenoh 管理功能
pub struct ZenohManager;

impl ZenohManager {
    /// 启动 zenohd
    pub async fn run_zenohd() -> Result<()> {
        if Self::is_zenohd_running()? {
            info!("zenohd is already running.");
        } else {
            info!("zenohd not running, starting it...");
            let cmd = which::which("zenohd")?;
            let port = Self::get_free_port_in_range()
                .await
                .ok_or_else(|| eyre!("No available port in range 7444-7500"))?;

            let output = Command::new(cmd)
                .args(&["-l", &format!("tcp/0.0.0.0:{}", port)])
                .spawn()?;

            info!("zenohd started with PID: {} on port {}", output.id(), port);
        }
        Ok(())
    }

    /// 检查 zenohd 是否在运行
    fn is_zenohd_running() -> Result<bool> {
        let output = Command::new("pgrep").args(&["-f", "zenohd"]).output()?;
        Ok(!output.stdout.is_empty())
    }

    /// 获取可用端口
    async fn get_free_port_in_range() -> Option<u16> {
        for port in 7444..=7500 {
            if Self::is_port_free(port).await {
                return Some(port);
            }
        }
        None
    }

    /// 检查端口是否可用
    async fn is_port_free(port: u16) -> bool {
        match TcpListener::bind(("0.0.0.0", port)).await {
            Ok(_) => true,
            Err(_) => false,
        }
    }

    /// 获取随机端口
    fn get_random_port() -> u16 {
        let mut rng = rand::rng();
        rng.random_range(7444..=7500)
    }

    /// 列出 Zenoh 主题
    pub async fn list_topics(session: &zenoh::Session, name: Option<String>) -> Result<Vec<String>> {
        let query_selector = match name {
            Some(n) => format!("{}/**", n),
            None => "*".to_string(),
        };

        let replies = session.get(&query_selector).await.map_err(|e| eyre!(e))?;
        let mut topics = Vec::new();

        while let Ok(reply) = replies.recv_async().await {
            match reply.result() {
                Ok(sample) => {
                    topics.push(sample.key_expr().to_string());
                }
                Err(e) => {
                    error!("Error receiving reply: {}", e);
                }
            }
        }

        Ok(topics)
    }
}
