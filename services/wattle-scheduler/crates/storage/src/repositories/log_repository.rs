use crate::{
    DB,
    model::WorkerLogEntity,
};
use eyre::Result;
use sqlx::{Row, query};

pub struct LogRepository<'a> {
    db: &'a DB,
}

impl<'a> LogRepository<'a> {
    pub fn new(db: &'a DB) -> Self {
        Self { db }
    }

    /// 插入工作者日志
    pub async fn insert_worker_log(
        &self,
        workflow_name: &str,
        worker_name: &str,
        log_type: &str,
        file_path: &str,
    ) -> Result<()> {
        query(
            "INSERT INTO worker_logs (workflow_name, worker_name, log_type, file_path, created_at) VALUES (?1, ?2, ?3, ?4, ?5)",
        )
        .bind(workflow_name)
        .bind(worker_name)
        .bind(log_type)
        .bind(file_path)
        .bind(chrono::Utc::now().to_rfc3339())
        .execute(self.db)
        .await?;

        Ok(())
    }

    /// 获取工作者日志
    pub async fn get_worker_logs(
        &self,
        workflow_name: &str,
        worker_name: &str,
    ) -> Result<Vec<WorkerLogEntity>> {
        let mut logs = Vec::new();
        
        let rows = query(
            "SELECT id, workflow_name, worker_name, log_type, file_path, created_at FROM worker_logs WHERE workflow_name = ? AND worker_name = ? ORDER BY created_at",
        )
        .bind(workflow_name)
        .bind(worker_name)
        .fetch_all(self.db)
        .await?;

        for row in rows {
            logs.push(WorkerLogEntity {
                id: row.get("id"),
                workflow_name: row.get("workflow_name"),
                worker_name: row.get("worker_name"),
                log_type: row.get("log_type"),
                file_path: row.get("file_path"),
                created_at: row.get("created_at"),
            });
        }

        Ok(logs)
    }
}
