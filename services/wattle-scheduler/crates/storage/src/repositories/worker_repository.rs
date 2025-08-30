use crate::{
    DB,
    model::WorkerEntity,
};
use core::{Worker, WorkerStatus};
use eyre::Result;
use sqlx::{Row, query, query_as};

pub struct WorkerRepository<'a> {
    db: &'a DB,
}

impl<'a> WorkerRepository<'a> {
    pub fn new(db: &'a DB) -> Self {
        Self { db }
    }

    /// 插入工作者
    pub async fn insert_workers(&self, workers: &[Worker]) -> Result<()> {
        for worker in workers {
            let entity = WorkerEntity::from(worker);
            query(
                "INSERT INTO workers (workflow_name, name, command, args, working_dir, env_vars, inputs, outputs, status, created_at) 
                 VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10)",
            )
            .bind(&entity.workflow_name)
            .bind(&entity.name)
            .bind(&entity.command)
            .bind(&entity.args)
            .bind(&entity.working_dir)
            .bind(&entity.env_vars)
            .bind(&entity.inputs)
            .bind(&entity.outputs)
            .bind(&entity.status)
            .bind(&entity.created_at)
            .execute(self.db)
            .await?;
        }

        Ok(())
    }

    /// 根据工作流名称和工作者名称获取工作者
    pub async fn get_worker(&self, workflow_name: &str, worker_name: &str) -> Result<Option<Worker>> {
        let row: Option<sqlx::sqlite::SqliteRow> = query(
            "SELECT workflow_name, name, command, args, working_dir, env_vars, inputs, outputs, status, error_message, created_at, deleted_at, started_at, completed_at 
             FROM workers 
             WHERE workflow_name = ? AND name = ? AND deleted_at IS NULL",
        )
        .bind(workflow_name)
        .bind(worker_name)
        .fetch_optional(self.db)
        .await?;

        if let Some(row) = row {
            let entity = WorkerEntity {
                workflow_name: row.get("workflow_name"),
                name: row.get("name"),
                command: row.get("command"),
                args: row.get("args"),
                working_dir: row.get("working_dir"),
                env_vars: row.get("env_vars"),
                inputs: row.get("inputs"),
                outputs: row.get("outputs"),
                status: row.get("status"),
                error_message: row.get("error_message"),
                created_at: row.get("created_at"),
                deleted_at: row.get("deleted_at"),
                started_at: row.get("started_at"),
                completed_at: row.get("completed_at"),
            };

            return Ok(Some(entity.into()));
        }

        Ok(None)
    }

    /// 根据工作流名称获取所有工作者
    pub async fn get_workers_by_workflow(&self, workflow_name: &str) -> Result<Vec<WorkerEntity>> {
        let rows: Vec<WorkerEntity> = query_as(
            "SELECT workflow_name, name, command, args, working_dir, env_vars, inputs, outputs, status, error_message, created_at, deleted_at, started_at, completed_at 
             FROM workers 
             WHERE workflow_name = ? AND deleted_at IS NULL 
             ORDER BY created_at",
        )
        .bind(workflow_name)
        .fetch_all(self.db)
        .await?;

        Ok(rows)
    }

    /// 检查工作者是否存在
    pub async fn worker_exists(&self, workflow_name: &str, worker_name: &str) -> Result<bool> {
        let row = query("SELECT COUNT(*) as count FROM workers WHERE workflow_name = ? AND name = ? AND deleted_at IS NULL")
            .bind(workflow_name)
            .bind(worker_name)
            .fetch_one(self.db)
            .await?;
        let count: i64 = row.get("count");
        Ok(count > 0)
    }

    /// 更新工作者状态
    pub async fn update_worker_status(
        &self,
        workflow_name: &str,
        worker_name: &str,
        status: WorkerStatus,
    ) -> Result<()> {
        let now = chrono::Utc::now().to_rfc3339();
        let status_str = status.to_string();
        
        let (started_at, completed_at) = match status {
            WorkerStatus::Running => (Some(now.clone()), None),
            WorkerStatus::Completed | WorkerStatus::Failed | WorkerStatus::Cancelled => (None, Some(now.clone())),
            _ => (None, None),
        };

        let mut query_str = "UPDATE workers SET status = ?".to_string();
        let mut bind_values = vec![status_str, workflow_name.to_string(), worker_name.to_string()];
        let mut param_count = 1;
        
        if let Some(ref started) = started_at {
            param_count += 1;
            query_str.push_str(&format!(", started_at = ?{}", param_count));
            bind_values.insert(1, started.clone());
        }
        
        if let Some(ref completed) = completed_at {
            param_count += 1;
            query_str.push_str(&format!(", completed_at = ?{}", param_count));
            bind_values.insert(if started_at.is_some() { 2 } else { 1 }, completed.clone());
        }
        
        query_str.push_str(&format!(" WHERE workflow_name = ?{} AND name = ?{}", param_count + 1, param_count + 2));

        let mut query_builder = query(&query_str);
        for value in bind_values {
            query_builder = query_builder.bind(value);
        }
        
        query_builder.execute(self.db).await?;
        Ok(())
    }

    /// 更新工作者错误信息
    pub async fn update_worker_error(
        &self,
        workflow_name: &str,
        worker_name: &str,
        error_message: &str,
    ) -> Result<()> {
        query(
            "UPDATE workers SET error_message = ?, status = 'failed', completed_at = ? WHERE workflow_name = ? AND name = ?",
        )
        .bind(error_message)
        .bind(chrono::Utc::now().to_rfc3339())
        .bind(workflow_name)
        .bind(worker_name)
        .execute(self.db)
        .await?;

        Ok(())
    }

    /// 删除工作者 (软删除)
    pub async fn delete_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        query("UPDATE workers SET deleted_at = ? WHERE workflow_name = ? AND name = ?")
            .bind(chrono::Utc::now().to_rfc3339())
            .bind(workflow_name)
            .bind(worker_name)
            .execute(self.db)
            .await?;

        Ok(())
    }
}
