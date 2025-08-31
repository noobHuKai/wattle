//! Metrics and monitoring for wattle-rs
//! 
//! This module provides comprehensive metrics collection and reporting
//! for monitoring the performance and health of the Wattle system.

#[cfg(feature = "metrics")]
use metrics::{counter, gauge, histogram, Counter, Gauge, Histogram};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Comprehensive metrics collector for Wattle
#[derive(Debug, Clone)]
pub struct WattleMetrics {
    // Message metrics
    pub messages_published: Arc<AtomicU64>,
    pub messages_delivered: Arc<AtomicU64>,
    pub messages_failed: Arc<AtomicU64>,
    pub messages_expired: Arc<AtomicU64>,
    
    // Request/Response metrics
    pub requests_sent: Arc<AtomicU64>,
    pub responses_received: Arc<AtomicU64>,
    pub requests_timed_out: Arc<AtomicU64>,
    pub request_errors: Arc<AtomicU64>,
    
    // Subscription metrics
    pub active_subscriptions: Arc<AtomicUsize>,
    pub total_subscriptions: Arc<AtomicU64>,
    pub subscription_errors: Arc<AtomicU64>,
    
    // Callback metrics
    pub callbacks_executed: Arc<AtomicU64>,
    pub callback_failures: Arc<AtomicU64>,
    pub callback_timeouts: Arc<AtomicU64>,
    
    // Performance metrics
    pub message_processing_latency: Arc<AtomicU64>, // microseconds
    pub request_response_latency: Arc<AtomicU64>,  // microseconds
    pub callback_execution_time: Arc<AtomicU64>,   // microseconds
    
    // Resource metrics
    pub memory_usage: Arc<AtomicU64>,
    pub active_threads: Arc<AtomicUsize>,
    pub queue_depth: Arc<AtomicUsize>,
    pub connection_count: Arc<AtomicUsize>,
    
    // Error metrics by category
    pub data_format_errors: Arc<AtomicU64>,
    pub network_errors: Arc<AtomicU64>,
    pub timeout_errors: Arc<AtomicU64>,
    pub resource_errors: Arc<AtomicU64>,
    
    // Health metrics
    pub uptime_seconds: Arc<AtomicU64>,
    pub last_heartbeat: Arc<AtomicU64>,
    pub health_check_failures: Arc<AtomicU64>,
    
    start_time: Instant,
}

impl WattleMetrics {
    /// Create a new metrics instance
    pub fn new() -> Self {
        let start_time = Instant::now();
        
        Self {
            messages_published: Arc::new(AtomicU64::new(0)),
            messages_delivered: Arc::new(AtomicU64::new(0)),
            messages_failed: Arc::new(AtomicU64::new(0)),
            messages_expired: Arc::new(AtomicU64::new(0)),
            
            requests_sent: Arc::new(AtomicU64::new(0)),
            responses_received: Arc::new(AtomicU64::new(0)),
            requests_timed_out: Arc::new(AtomicU64::new(0)),
            request_errors: Arc::new(AtomicU64::new(0)),
            
            active_subscriptions: Arc::new(AtomicUsize::new(0)),
            total_subscriptions: Arc::new(AtomicU64::new(0)),
            subscription_errors: Arc::new(AtomicU64::new(0)),
            
            callbacks_executed: Arc::new(AtomicU64::new(0)),
            callback_failures: Arc::new(AtomicU64::new(0)),
            callback_timeouts: Arc::new(AtomicU64::new(0)),
            
            message_processing_latency: Arc::new(AtomicU64::new(0)),
            request_response_latency: Arc::new(AtomicU64::new(0)),
            callback_execution_time: Arc::new(AtomicU64::new(0)),
            
            memory_usage: Arc::new(AtomicU64::new(0)),
            active_threads: Arc::new(AtomicUsize::new(0)),
            queue_depth: Arc::new(AtomicUsize::new(0)),
            connection_count: Arc::new(AtomicUsize::new(0)),
            
            data_format_errors: Arc::new(AtomicU64::new(0)),
            network_errors: Arc::new(AtomicU64::new(0)),
            timeout_errors: Arc::new(AtomicU64::new(0)),
            resource_errors: Arc::new(AtomicU64::new(0)),
            
            uptime_seconds: Arc::new(AtomicU64::new(0)),
            last_heartbeat: Arc::new(AtomicU64::new(0)),
            health_check_failures: Arc::new(AtomicU64::new(0)),
            
            start_time,
        }
    }
    
    // Message metrics methods
    pub fn increment_messages_published(&self) {
        self.messages_published.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_messages_published_total").increment(1);
    }
    
    pub fn increment_messages_delivered(&self) {
        self.messages_delivered.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_messages_delivered_total").increment(1);
    }
    
    pub fn increment_messages_failed(&self) {
        self.messages_failed.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_messages_failed_total").increment(1);
    }
    
    pub fn increment_messages_expired(&self) {
        self.messages_expired.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_messages_expired_total").increment(1);
    }
    
    // Request/Response metrics methods
    pub fn increment_requests_sent(&self) {
        self.requests_sent.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_requests_sent_total").increment(1);
    }
    
    pub fn increment_responses_received(&self) {
        self.responses_received.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_responses_received_total").increment(1);
    }
    
    pub fn increment_requests_timed_out(&self) {
        self.requests_timed_out.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_requests_timed_out_total").increment(1);
    }
    
    // Subscription metrics methods
    pub fn increment_active_subscriptions(&self) {
        self.active_subscriptions.fetch_add(1, Ordering::Relaxed);
        self.total_subscriptions.fetch_add(1, Ordering::Relaxed);
        
        #[cfg(feature = "metrics")]
        {
            gauge!("wattle_active_subscriptions").set(self.active_subscriptions.load(Ordering::Relaxed) as f64);
            counter!("wattle_subscriptions_total").increment(1);
        }
    }
    
    pub fn decrement_active_subscriptions(&self) {
        self.active_subscriptions.fetch_sub(1, Ordering::Relaxed);
        
        #[cfg(feature = "metrics")]
        gauge!("wattle_active_subscriptions").set(self.active_subscriptions.load(Ordering::Relaxed) as f64);
    }
    
    // Callback metrics methods
    pub fn increment_callbacks_executed(&self) {
        self.callbacks_executed.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_callbacks_executed_total").increment(1);
    }
    
    pub fn increment_callback_failures(&self) {
        self.callback_failures.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_callback_failures_total").increment(1);
    }
    
    pub fn increment_callback_timeouts(&self) {
        self.callback_timeouts.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_callback_timeouts_total").increment(1);
    }
    
    // Latency tracking methods
    pub fn record_message_processing_latency(&self, latency: Duration) {
        let micros = latency.as_micros() as u64;
        self.message_processing_latency.store(micros, Ordering::Relaxed);
        
        #[cfg(feature = "metrics")]
        histogram!("wattle_message_processing_duration_us").record(micros as f64);
    }
    
    pub fn record_request_response_latency(&self, latency: Duration) {
        let micros = latency.as_micros() as u64;
        self.request_response_latency.store(micros, Ordering::Relaxed);
        
        #[cfg(feature = "metrics")]
        histogram!("wattle_request_response_duration_us").record(micros as f64);
    }
    
    pub fn record_callback_execution_time(&self, duration: Duration) {
        let micros = duration.as_micros() as u64;
        self.callback_execution_time.store(micros, Ordering::Relaxed);
        
        #[cfg(feature = "metrics")]
        histogram!("wattle_callback_execution_duration_us").record(micros as f64);
    }
    
    // Resource metrics methods
    pub fn update_memory_usage(&self, bytes: u64) {
        self.memory_usage.store(bytes, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        gauge!("wattle_memory_usage_bytes").set(bytes as f64);
    }
    
    pub fn update_queue_depth(&self, depth: usize) {
        self.queue_depth.store(depth, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        gauge!("wattle_queue_depth").set(depth as f64);
    }
    
    pub fn update_connection_count(&self, count: usize) {
        self.connection_count.store(count, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        gauge!("wattle_connection_count").set(count as f64);
    }
    
    // Error metrics methods
    pub fn increment_error_by_type(&self, error_type: &str) {
        match error_type {
            "data_format" => {
                self.data_format_errors.fetch_add(1, Ordering::Relaxed);
            }
            "network" => {
                self.network_errors.fetch_add(1, Ordering::Relaxed);
            }
            "timeout" => {
                self.timeout_errors.fetch_add(1, Ordering::Relaxed);
            }
            "resource" => {
                self.resource_errors.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }
        
        #[cfg(feature = "metrics")]
        counter!("wattle_errors_total", "type" => error_type).increment(1);
    }
    
    // Health metrics methods
    pub fn update_uptime(&self) {
        let uptime = self.start_time.elapsed().as_secs();
        self.uptime_seconds.store(uptime, Ordering::Relaxed);
        
        #[cfg(feature = "metrics")]
        gauge!("wattle_uptime_seconds").set(uptime as f64);
    }
    
    pub fn heartbeat(&self) {
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        
        self.last_heartbeat.store(timestamp, Ordering::Relaxed);
        
        #[cfg(feature = "metrics")]
        gauge!("wattle_last_heartbeat_timestamp").set(timestamp as f64);
    }
    
    pub fn increment_health_check_failures(&self) {
        self.health_check_failures.fetch_add(1, Ordering::Relaxed);
        #[cfg(feature = "metrics")]
        counter!("wattle_health_check_failures_total").increment(1);
    }
    
    // Getter methods for current values
    pub fn get_messages_published(&self) -> u64 {
        self.messages_published.load(Ordering::Relaxed)
    }
    
    pub fn get_messages_delivered(&self) -> u64 {
        self.messages_delivered.load(Ordering::Relaxed)
    }
    
    pub fn get_active_subscriptions(&self) -> usize {
        self.active_subscriptions.load(Ordering::Relaxed)
    }
    
    pub fn get_callback_success_rate(&self) -> f64 {
        let executed = self.callbacks_executed.load(Ordering::Relaxed);
        let failed = self.callback_failures.load(Ordering::Relaxed);
        
        if executed == 0 {
            return 0.0;
        }
        
        let successful = executed.saturating_sub(failed);
        successful as f64 / executed as f64
    }
    
    pub fn get_request_success_rate(&self) -> f64 {
        let sent = self.requests_sent.load(Ordering::Relaxed);
        let errors = self.request_errors.load(Ordering::Relaxed);
        let timeouts = self.requests_timed_out.load(Ordering::Relaxed);
        
        if sent == 0 {
            return 0.0;
        }
        
        let successful = sent.saturating_sub(errors + timeouts);
        successful as f64 / sent as f64
    }
    
    pub fn get_average_message_latency(&self) -> Duration {
        Duration::from_micros(self.message_processing_latency.load(Ordering::Relaxed))
    }
    
    // Health status
    pub fn is_healthy(&self) -> bool {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        
        let last_heartbeat = self.last_heartbeat.load(Ordering::Relaxed);
        
        // Consider healthy if heartbeat was within last 60 seconds
        now - last_heartbeat < 60
    }
    
    /// Get a comprehensive metrics snapshot
    pub fn snapshot(&self) -> MetricsSnapshot {
        self.update_uptime();
        
        MetricsSnapshot {
            messages_published: self.messages_published.load(Ordering::Relaxed),
            messages_delivered: self.messages_delivered.load(Ordering::Relaxed),
            messages_failed: self.messages_failed.load(Ordering::Relaxed),
            messages_expired: self.messages_expired.load(Ordering::Relaxed),
            
            requests_sent: self.requests_sent.load(Ordering::Relaxed),
            responses_received: self.responses_received.load(Ordering::Relaxed),
            requests_timed_out: self.requests_timed_out.load(Ordering::Relaxed),
            request_errors: self.request_errors.load(Ordering::Relaxed),
            
            active_subscriptions: self.active_subscriptions.load(Ordering::Relaxed),
            total_subscriptions: self.total_subscriptions.load(Ordering::Relaxed),
            
            callbacks_executed: self.callbacks_executed.load(Ordering::Relaxed),
            callback_failures: self.callback_failures.load(Ordering::Relaxed),
            callback_timeouts: self.callback_timeouts.load(Ordering::Relaxed),
            
            message_processing_latency_us: self.message_processing_latency.load(Ordering::Relaxed),
            request_response_latency_us: self.request_response_latency.load(Ordering::Relaxed),
            callback_execution_time_us: self.callback_execution_time.load(Ordering::Relaxed),
            
            memory_usage_bytes: self.memory_usage.load(Ordering::Relaxed),
            queue_depth: self.queue_depth.load(Ordering::Relaxed),
            connection_count: self.connection_count.load(Ordering::Relaxed),
            
            uptime_seconds: self.uptime_seconds.load(Ordering::Relaxed),
            is_healthy: self.is_healthy(),
        }
    }
}

impl Default for WattleMetrics {
    fn default() -> Self {
        Self::new()
    }
}

/// Immutable snapshot of metrics at a point in time
#[derive(Debug, Clone)]
pub struct MetricsSnapshot {
    pub messages_published: u64,
    pub messages_delivered: u64,
    pub messages_failed: u64,
    pub messages_expired: u64,
    
    pub requests_sent: u64,
    pub responses_received: u64,
    pub requests_timed_out: u64,
    pub request_errors: u64,
    
    pub active_subscriptions: usize,
    pub total_subscriptions: u64,
    
    pub callbacks_executed: u64,
    pub callback_failures: u64,
    pub callback_timeouts: u64,
    
    pub message_processing_latency_us: u64,
    pub request_response_latency_us: u64,
    pub callback_execution_time_us: u64,
    
    pub memory_usage_bytes: u64,
    pub queue_depth: usize,
    pub connection_count: usize,
    
    pub uptime_seconds: u64,
    pub is_healthy: bool,
}

impl MetricsSnapshot {
    /// Calculate message success rate
    pub fn message_success_rate(&self) -> f64 {
        if self.messages_published == 0 {
            return 0.0;
        }
        
        let successful = self.messages_delivered;
        successful as f64 / self.messages_published as f64
    }
    
    /// Calculate request success rate
    pub fn request_success_rate(&self) -> f64 {
        if self.requests_sent == 0 {
            return 0.0;
        }
        
        let successful = self.requests_sent
            .saturating_sub(self.request_errors + self.requests_timed_out);
        successful as f64 / self.requests_sent as f64
    }
    
    /// Calculate callback success rate
    pub fn callback_success_rate(&self) -> f64 {
        if self.callbacks_executed == 0 {
            return 0.0;
        }
        
        let successful = self.callbacks_executed.saturating_sub(self.callback_failures);
        successful as f64 / self.callbacks_executed as f64
    }
}

/// Metrics reporter that periodically logs or exports metrics
pub struct MetricsReporter {
    metrics: WattleMetrics,
    interval: Duration,
    last_report: Instant,
}

impl MetricsReporter {
    /// Create a new metrics reporter
    pub fn new(metrics: WattleMetrics, interval: Duration) -> Self {
        Self {
            metrics,
            interval,
            last_report: Instant::now(),
        }
    }
    
    /// Check if it's time to report metrics
    pub fn should_report(&self) -> bool {
        self.last_report.elapsed() >= self.interval
    }
    
    /// Generate and return a metrics report
    pub fn report(&mut self) -> MetricsSnapshot {
        self.last_report = Instant::now();
        self.metrics.snapshot()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;
    
    #[test]
    fn test_metrics_creation() {
        let metrics = WattleMetrics::new();
        assert_eq!(metrics.get_messages_published(), 0);
        assert_eq!(metrics.get_active_subscriptions(), 0);
    }
    
    #[test]
    fn test_message_metrics() {
        let metrics = WattleMetrics::new();
        
        metrics.increment_messages_published();
        metrics.increment_messages_delivered();
        
        assert_eq!(metrics.get_messages_published(), 1);
        assert_eq!(metrics.get_messages_delivered(), 1);
    }
    
    #[test]
    fn test_subscription_metrics() {
        let metrics = WattleMetrics::new();
        
        metrics.increment_active_subscriptions();
        metrics.increment_active_subscriptions();
        
        assert_eq!(metrics.get_active_subscriptions(), 2);
        
        metrics.decrement_active_subscriptions();
        assert_eq!(metrics.get_active_subscriptions(), 1);
    }
    
    #[test]
    fn test_success_rates() {
        let metrics = WattleMetrics::new();
        
        // Test callback success rate
        metrics.callbacks_executed.store(10, Ordering::Relaxed);
        metrics.callback_failures.store(2, Ordering::Relaxed);
        
        assert_eq!(metrics.get_callback_success_rate(), 0.8);
        
        // Test request success rate
        metrics.requests_sent.store(20, Ordering::Relaxed);
        metrics.request_errors.store(1, Ordering::Relaxed);
        metrics.requests_timed_out.store(1, Ordering::Relaxed);
        
        assert_eq!(metrics.get_request_success_rate(), 0.9);
    }
    
    #[test]
    fn test_metrics_snapshot() {
        let metrics = WattleMetrics::new();
        
        metrics.increment_messages_published();
        metrics.increment_active_subscriptions();
        
        let snapshot = metrics.snapshot();
        
        assert_eq!(snapshot.messages_published, 1);
        assert_eq!(snapshot.active_subscriptions, 1);
        assert_eq!(snapshot.message_success_rate(), 0.0); // No delivered messages yet
    }
    
    #[test]
    fn test_health_status() {
        let metrics = WattleMetrics::new();
        
        // Initially not healthy (no heartbeat)
        assert!(!metrics.is_healthy());
        
        // After heartbeat, should be healthy
        metrics.heartbeat();
        assert!(metrics.is_healthy());
    }
    
    #[test]
    fn test_metrics_reporter() {
        let metrics = WattleMetrics::new();
        let mut reporter = MetricsReporter::new(metrics, Duration::from_millis(100));
        
        assert!(reporter.should_report());
        
        let _snapshot = reporter.report();
        assert!(!reporter.should_report());
        
        // Wait for interval
        thread::sleep(Duration::from_millis(150));
        assert!(reporter.should_report());
    }
}
