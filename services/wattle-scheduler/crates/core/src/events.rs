use tokio::sync::broadcast;
use crate::{EventPublisher, EventMessage, WattleError};
use async_trait::async_trait;

/// 基于 broadcast channel 的事件发布器实现
#[derive(Debug, Clone)]
pub struct BroadcastEventPublisher {
    sender: broadcast::Sender<EventMessage>,
}

impl BroadcastEventPublisher {
    /// 创建新的事件发布器
    /// capacity: channel 容量，默认 1000
    pub fn new(capacity: usize) -> Self {
        let (sender, _) = broadcast::channel(capacity);
        Self { sender }
    }

    /// 获取订阅者数量
    pub fn subscriber_count(&self) -> usize {
        self.sender.receiver_count()
    }

    /// 发布通用事件
    async fn publish_event(&self, event: EventMessage) -> Result<(), WattleError> {
        match self.sender.send(event) {
            Ok(_) => Ok(()),
            Err(broadcast::error::SendError(_)) => {
                // 没有活跃的接收者，这是正常的
                Ok(())
            }
        }
    }
}

#[async_trait]
impl EventPublisher for BroadcastEventPublisher {
    async fn publish_workflow_event(&self, workflow_name: &str, event_type: &str, data: &str) -> Result<(), WattleError> {
        let event = EventMessage::new(event_type, workflow_name, data);
        self.publish_event(event).await
    }

    async fn publish_worker_event(&self, worker_name: &str, workflow_name: &str, event_type: &str, data: &str) -> Result<(), WattleError> {
        let event = EventMessage::new(event_type, workflow_name, data)
            .with_worker(worker_name);
        self.publish_event(event).await
    }

    async fn publish_log_event(&self, worker_name: &str, workflow_name: &str, log_line: &str) -> Result<(), WattleError> {
        let event = EventMessage::new("log", workflow_name, log_line)
            .with_worker(worker_name);
        self.publish_event(event).await
    }

    async fn subscribe_events(&self) -> Result<broadcast::Receiver<EventMessage>, WattleError> {
        Ok(self.sender.subscribe())
    }
}

/// 带过滤的事件订阅器
pub struct FilteredEventSubscriber {
    receiver: broadcast::Receiver<EventMessage>,
    workflow_filter: Option<String>,
    worker_filter: Option<String>,
    event_type_filter: Option<String>,
}

impl FilteredEventSubscriber {
    pub fn new(
        receiver: broadcast::Receiver<EventMessage>,
        workflow_filter: Option<String>,
        worker_filter: Option<String>,
        event_type_filter: Option<String>,
    ) -> Self {
        Self {
            receiver,
            workflow_filter,
            worker_filter,
            event_type_filter,
        }
    }

    /// 接收下一个匹配过滤条件的事件
    pub async fn recv(&mut self) -> Result<EventMessage, broadcast::error::RecvError> {
        loop {
            let event = self.receiver.recv().await?;
            
            // 应用过滤条件
            if let Some(ref workflow_filter) = self.workflow_filter {
                if event.workflow_name != *workflow_filter {
                    continue;
                }
            }

            if let Some(ref worker_filter) = self.worker_filter {
                if event.worker_name.as_ref() != Some(worker_filter) {
                    continue;
                }
            }

            if let Some(ref event_type_filter) = self.event_type_filter {
                if event.event_type != *event_type_filter {
                    continue;
                }
            }

            return Ok(event);
        }
    }

    /// 非阻塞接收
    pub fn try_recv(&mut self) -> Result<Option<EventMessage>, broadcast::error::TryRecvError> {
        loop {
            match self.receiver.try_recv() {
                Ok(event) => {
                    // 应用过滤条件
                    if let Some(ref workflow_filter) = self.workflow_filter {
                        if event.workflow_name != *workflow_filter {
                            continue;
                        }
                    }

                    if let Some(ref worker_filter) = self.worker_filter {
                        if event.worker_name.as_ref() != Some(worker_filter) {
                            continue;
                        }
                    }

                    if let Some(ref event_type_filter) = self.event_type_filter {
                        if event.event_type != *event_type_filter {
                            continue;
                        }
                    }

                    return Ok(Some(event));
                }
                Err(broadcast::error::TryRecvError::Empty) => return Ok(None),
                Err(err) => return Err(err),
            }
        }
    }
}
