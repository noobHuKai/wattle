use crate::{
    DB,
    model::{WorkerEntity, WorkflowEntity, WorkerLogEntity},
};
use core::{Worker, Workflow, WorkerStatus, DatabaseConfig};
use eyre::Result;
use sqlx::{Row, query, query_as};

pub struct Repositories {
    db: DB,
}

impl Repositories {
    pub async fn new(db_url: Option<String>, db_config: Option<DatabaseConfig>) -> Result<Self> {
        let db = crate::init_database(db_url, db_config).await?;
        Ok(Self { db })
    }

    // ========== Workflow 操作 ==========

    /// 插入工作流
    pub async fn insert_workflow(&self, workflow: &Workflow) -> Result<()> {
        let entity = WorkflowEntity::from(workflow);
        query(
            "INSERT INTO workflows (name, working_dir, status, created_at) VALUES (?1, ?2, ?3, ?4)",
        )
        .bind(&entity.name)
        .bind(&entity.working_dir)
        .bind(&entity.status)
        .bind(&entity.created_at)
        .execute(&self.db)
        .await?;

        Ok(())
    }

    /// 根据名称获取工作流
    pub async fn get_workflow(&self, name: &str) -> Result<Option<WorkflowEntity>> {
        let row: Option<WorkflowEntity> = query_as(
            "SELECT name, working_dir, status, created_at, deleted_at, started_at, completed_at 
             FROM workflows 
             WHERE name = ? AND deleted_at IS NULL",
        )
        .bind(name)
        .fetch_optional(&self.db)
        .await?;
        Ok(row)
    }

    /// 获取所有工作流
    pub async fn get_all_workflows(&self) -> Result<Vec<WorkflowEntity>> {
        let rows: Vec<WorkflowEntity> = query_as(
            "SELECT name, working_dir, status, created_at, deleted_at, started_at, completed_at 
             FROM workflows 
             WHERE deleted_at IS NULL",
        )
        .fetch_all(&self.db)
        .await?;

        Ok(rows)
    }

    /// 获取工作流列表（支持分页和过滤）
    pub async fn get_workflows_paged(
        &self,
        page: u64,
        page_size: u64,
        status_filter: Option<&str>,
        sort_by: &str,
        order: &str,
    ) -> Result<(Vec<WorkflowEntity>, u64)> {
        let offset = (page - 1) * page_size;
        
        // 构建查询条件
        let mut where_clause = "WHERE deleted_at IS NULL".to_string();
        let mut count_params = Vec::new();
        let mut query_params = Vec::new();
        
        if let Some(status) = status_filter {
            where_clause.push_str(" AND status = ?");
            count_params.push(status);
            query_params.push(status);
        }

        // 获取总数
        let count_query = format!("SELECT COUNT(*) as count FROM workflows {}", where_clause);
        let mut count_query_builder = query(&count_query);
        for param in &count_params {
            count_query_builder = count_query_builder.bind(param);
        }
        
        let total_row = count_query_builder.fetch_one(&self.db).await?;
        let total: i64 = total_row.get("count");

        // 构建排序子句
        let order_clause = match sort_by {
            "name" => format!("ORDER BY name {}", order),
            "status" => format!("ORDER BY status {}", order),
            "started_at" => format!("ORDER BY started_at {}", order),
            "completed_at" => format!("ORDER BY completed_at {}", order),
            _ => format!("ORDER BY created_at {}", order), // 默认按创建时间排序
        };

        // 获取分页数据
        let data_query = format!(
            "SELECT name, working_dir, status, created_at, deleted_at, started_at, completed_at 
             FROM workflows {} {} LIMIT ? OFFSET ?",
            where_clause, order_clause
        );
        
        let mut data_query_builder = query_as(&data_query);
        for param in &query_params {
            data_query_builder = data_query_builder.bind(param);
        }
        data_query_builder = data_query_builder.bind(page_size as i64).bind(offset as i64);
        
        let rows: Vec<WorkflowEntity> = data_query_builder.fetch_all(&self.db).await?;

        Ok((rows, total as u64))
    }

    /// 检查工作流是否存在
    pub async fn workflow_exists(&self, name: &str) -> Result<bool> {
        let row = query("SELECT COUNT(*) as count FROM workflows WHERE name = ? AND deleted_at IS NULL")
            .bind(name)
            .fetch_one(&self.db)
            .await?;
        let count: i64 = row.get("count");
        Ok(count > 0)
    }

    /// 更新工作流状态
    pub async fn update_workflow_status(&self, name: &str, status: WorkerStatus) -> Result<()> {
        let now = chrono::Utc::now().to_rfc3339();
        let status_str = status.to_string();
        
        let (started_at, completed_at) = match status {
            WorkerStatus::Running => (Some(now.clone()), None),
            WorkerStatus::Completed | WorkerStatus::Failed | WorkerStatus::Cancelled => (None, Some(now.clone())),
            _ => (None, None),
        };

        let mut query_str = "UPDATE workflows SET status = ?".to_string();
        let mut bind_values = vec![status_str];
        
        if let Some(started) = started_at {
            query_str.push_str(", started_at = ?");
            bind_values.push(started);
        }
        
        if let Some(completed) = completed_at {
            query_str.push_str(", completed_at = ?");
            bind_values.push(completed);
        }
        
        query_str.push_str(" WHERE name = ?");
        bind_values.push(name.to_string());

        let mut query_builder = query(&query_str);
        for value in bind_values {
            query_builder = query_builder.bind(value);
        }
        
        query_builder.execute(&self.db).await?;
        Ok(())
    }

    /// 删除工作流 (软删除)
    pub async fn delete_workflow(&self, name: &str) -> Result<()> {
        let now = chrono::Utc::now().to_rfc3339();
        query("UPDATE workflows SET deleted_at = ? WHERE name = ?")
            .bind(&now)
            .bind(name)
            .execute(&self.db)
            .await?;

        // 同时软删除相关的工作者
        query("UPDATE workers SET deleted_at = ? WHERE workflow_name = ?")
            .bind(&now)
            .bind(name)
            .execute(&self.db)
            .await?;

        Ok(())
    }

    // ========== Worker 操作 ==========

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
            .execute(&self.db)
            .await?;
        }

        Ok(())
    }

    /// 根据工作流名称和工作者名称获取工作者
    pub async fn get_worker(&self, workflow_name: &str, worker_name: &str) -> Result<Option<Worker>> {
        let row: Option<sqlx::sqlite::SqliteRow> = query(
            "SELECT workflow_name, name, command, args, working_dir, env_vars, status, error_message, created_at, deleted_at, started_at, completed_at 
             FROM workers 
             WHERE workflow_name = ? AND name = ? AND deleted_at IS NULL",
        )
        .bind(workflow_name)
        .bind(worker_name)
        .fetch_optional(&self.db)
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
            "SELECT workflow_name, name, command, args, working_dir, env_vars, status, error_message, created_at, deleted_at, started_at, completed_at 
             FROM workers 
             WHERE workflow_name = ? AND deleted_at IS NULL 
             ORDER BY created_at",
        )
        .bind(workflow_name)
        .fetch_all(&self.db)
        .await?;

        Ok(rows)
    }

    /// 检查工作者是否存在
    pub async fn worker_exists(&self, workflow_name: &str, worker_name: &str) -> Result<bool> {
        let row = query("SELECT COUNT(*) as count FROM workers WHERE workflow_name = ? AND name = ? AND deleted_at IS NULL")
            .bind(workflow_name)
            .bind(worker_name)
            .fetch_one(&self.db)
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
        
        query_builder.execute(&self.db).await?;
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
        .execute(&self.db)
        .await?;

        Ok(())
    }

    /// 删除工作者 (软删除)
    pub async fn delete_worker(&self, workflow_name: &str, worker_name: &str) -> Result<()> {
        query("UPDATE workers SET deleted_at = ? WHERE workflow_name = ? AND name = ?")
            .bind(chrono::Utc::now().to_rfc3339())
            .bind(workflow_name)
            .bind(worker_name)
            .execute(&self.db)
            .await?;

        Ok(())
    }

    // ========== Log 操作 ==========

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
        .execute(&self.db)
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
        .fetch_all(&self.db)
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
