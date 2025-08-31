//! # Wattle-RS
//! 
//! A high-performance message passing and data processing system for Rust applications.
//! 
//! ## Features
//! 
//! - **Request/Reply Pattern**: Synchronous and asynchronous request handling
//! - **Publish/Subscribe Pattern**: Topic-based message broadcasting
//! - **Multiple Data Formats**: JSON, binary slices, and Apache Arrow support
//! - **Thread Safety**: All operations are thread-safe using Arc<Mutex<T>>
//! - **Async Support**: Built on tokio runtime with both sync and async APIs
//! - **Error Handling**: Unified error handling with eyre
//! 
//! ## Quick Start
//! 
//! ```rust
//! use wattle_rs::{Wattle, DataFormat};
//! 
//! #[tokio::main]
//! async fn main() -> wattle_rs::WattleResult<()> {
//!     let mut wattle = Wattle::new().await?;
//!     
//!     // Subscribe to a topic
//!     wattle.subscribe("test_topic", |data| {
//!         println!("Received: {:?}", data);
//!         Ok(())
//!     }).await?;
//!     
//!     // Publish a message
//!     wattle.publish("test_topic", DataFormat::Json(serde_json::json!({
//!         "message": "Hello, Wattle!"
//!     }))).await?;
//!     
//!     Ok(())
//! }
//! ```

pub mod core;
pub mod data_format;
pub mod messaging;
pub mod callbacks;
pub mod errors;
pub mod config;
pub mod metrics;

// Re-export commonly used types
pub use crate::core::Wattle;
pub use crate::data_format::DataFormat;
pub use crate::errors::{WattleError, WattleResult};
pub use crate::messaging::{Message, Request, Response, Topic};
pub use crate::callbacks::{SyncCallback, AsyncCallback};
pub use crate::config::WattleConfig;

#[cfg(feature = "metrics")]
pub use crate::metrics::WattleMetrics;

/// Current version of the wattle-rs library
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Initialize tracing for the wattle-rs library
/// 
/// This should be called once at the start of your application to enable
/// proper logging and tracing.
pub fn init_tracing() {
    use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};
    
    tracing_subscriber::registry()
        .with(tracing_subscriber::EnvFilter::new(
            std::env::var("RUST_LOG").unwrap_or_else(|_| "wattle_rs=info".into()),
        ))
        .with(tracing_subscriber::fmt::layer())
        .init();
}
