//! Error types and result handling for wattle-rs
//! 
//! This module defines all error types used throughout the library and provides
//! a unified Result type for consistent error handling.

use std::fmt;
use thiserror::Error;

/// Result type used throughout the wattle-rs library
pub type WattleResult<T> = eyre::Result<T>;

/// Comprehensive error types for wattle-rs operations
#[derive(Error, Debug)]
pub enum WattleError {
    /// Errors related to data format conversion or handling
    #[error("Data format error: {message}")]
    DataFormat { message: String },
    
    /// Errors related to messaging operations (pub/sub, req/reply)
    #[error("Messaging error: {message}")]
    Messaging { message: String },
    
    /// Errors related to callback execution
    #[error("Callback error: {message}")]
    Callback { message: String },
    
    /// Configuration-related errors
    #[error("Configuration error: {message}")]
    Configuration { message: String },
    
    /// Network or I/O related errors
    #[error("I/O error: {source}")]
    Io {
        #[from]
        source: std::io::Error,
    },
    
    /// JSON serialization/deserialization errors
    #[error("JSON error: {source}")]
    Json {
        #[from]
        source: serde_json::Error,
    },
    
    /// Arrow-related errors
    #[cfg(feature = "arrow")]
    #[error("Arrow error: {source}")]
    Arrow {
        #[from]
        source: arrow::error::ArrowError,
    },
    
    /// Timeout errors for request/reply operations
    #[error("Operation timed out after {timeout_ms}ms")]
    Timeout { timeout_ms: u64 },
    
    /// Subscriber not found errors
    #[error("Subscriber with ID '{subscriber_id}' not found")]
    SubscriberNotFound { subscriber_id: String },
    
    /// Topic-related errors
    #[error("Topic error: {message}")]
    Topic { message: String },
    
    /// Channel communication errors
    #[error("Channel error: {message}")]
    Channel { message: String },
    
    /// Resource exhaustion errors
    #[error("Resource exhausted: {resource}")]
    ResourceExhausted { resource: String },
    
    /// Thread synchronization errors
    #[error("Synchronization error: {message}")]
    Synchronization { message: String },
    
    /// Plugin-related errors
    #[error("Plugin error: {message}")]
    Plugin { message: String },
    
    /// General internal errors
    #[error("Internal error: {message}")]
    Internal { message: String },
}

impl WattleError {
    /// Create a new data format error
    pub fn data_format<S: Into<String>>(message: S) -> Self {
        Self::DataFormat {
            message: message.into(),
        }
    }
    
    /// Create a new messaging error
    pub fn messaging<S: Into<String>>(message: S) -> Self {
        Self::Messaging {
            message: message.into(),
        }
    }
    
    /// Create a new callback error
    pub fn callback<S: Into<String>>(message: S) -> Self {
        Self::Callback {
            message: message.into(),
        }
    }
    
    /// Create a new configuration error
    pub fn configuration<S: Into<String>>(message: S) -> Self {
        Self::Configuration {
            message: message.into(),
        }
    }
    
    /// Create a new timeout error
    pub fn timeout(timeout_ms: u64) -> Self {
        Self::Timeout { timeout_ms }
    }
    
    /// Create a new subscriber not found error
    pub fn subscriber_not_found<S: Into<String>>(subscriber_id: S) -> Self {
        Self::SubscriberNotFound {
            subscriber_id: subscriber_id.into(),
        }
    }
    
    /// Create a new topic error
    pub fn topic<S: Into<String>>(message: S) -> Self {
        Self::Topic {
            message: message.into(),
        }
    }
    
    /// Create a new channel error
    pub fn channel<S: Into<String>>(message: S) -> Self {
        Self::Channel {
            message: message.into(),
        }
    }
    
    /// Create a new resource exhausted error
    pub fn resource_exhausted<S: Into<String>>(resource: S) -> Self {
        Self::ResourceExhausted {
            resource: resource.into(),
        }
    }
    
    /// Create a new synchronization error
    pub fn synchronization<S: Into<String>>(message: S) -> Self {
        Self::Synchronization {
            message: message.into(),
        }
    }
    
    /// Create a new plugin error
    pub fn plugin<S: Into<String>>(message: S) -> Self {
        Self::Plugin {
            message: message.into(),
        }
    }
    
    /// Create a new internal error
    pub fn internal<S: Into<String>>(message: S) -> Self {
        Self::Internal {
            message: message.into(),
        }
    }
    
    /// Check if this error is recoverable
    pub fn is_recoverable(&self) -> bool {
        match self {
            Self::Timeout { .. } => true,
            Self::Channel { .. } => true,
            Self::ResourceExhausted { .. } => true,
            Self::Io { .. } => true,
            _ => false,
        }
    }
    
    /// Get the error category for metrics and monitoring
    pub fn category(&self) -> &'static str {
        match self {
            Self::DataFormat { .. } => "data_format",
            Self::Messaging { .. } => "messaging",
            Self::Callback { .. } => "callback",
            Self::Configuration { .. } => "configuration",
            Self::Io { .. } => "io",
            Self::Json { .. } => "json",
            #[cfg(feature = "arrow")]
            Self::Arrow { .. } => "arrow",
            Self::Timeout { .. } => "timeout",
            Self::SubscriberNotFound { .. } => "subscriber_not_found",
            Self::Topic { .. } => "topic",
            Self::Channel { .. } => "channel",
            Self::ResourceExhausted { .. } => "resource_exhausted",
            Self::Synchronization { .. } => "synchronization",
            Self::Plugin { .. } => "plugin",
            Self::Internal { .. } => "internal",
        }
    }
}

/// Helper macro for creating internal errors
#[macro_export]
macro_rules! internal_error {
    ($fmt:expr) => {
        $crate::errors::WattleError::internal($fmt)
    };
    ($fmt:expr, $($arg:tt)*) => {
        $crate::errors::WattleError::internal(format!($fmt, $($arg)*))
    };
}

/// Helper macro for creating messaging errors
#[macro_export]
macro_rules! messaging_error {
    ($fmt:expr) => {
        $crate::errors::WattleError::messaging($fmt)
    };
    ($fmt:expr, $($arg:tt)*) => {
        $crate::errors::WattleError::messaging(format!($fmt, $($arg)*))
    };
}

/// Helper macro for creating data format errors
#[macro_export]
macro_rules! data_format_error {
    ($fmt:expr) => {
        $crate::errors::WattleError::data_format($fmt)
    };
    ($fmt:expr, $($arg:tt)*) => {
        $crate::errors::WattleError::data_format(format!($fmt, $($arg)*))
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_error_creation() {
        let err = WattleError::messaging("test message");
        assert_eq!(err.category(), "messaging");
        assert!(!err.is_recoverable());
        
        let timeout_err = WattleError::timeout(5000);
        assert_eq!(timeout_err.category(), "timeout");
        assert!(timeout_err.is_recoverable());
    }
    
    #[test]
    fn test_error_macros() {
        let err = internal_error!("test error");
        assert_eq!(err.category(), "internal");
        
        let msg_err = messaging_error!("test {} error", "messaging");
        assert_eq!(msg_err.category(), "messaging");
        
        let data_err = data_format_error!("invalid format");
        assert_eq!(data_err.category(), "data_format");
    }
}
