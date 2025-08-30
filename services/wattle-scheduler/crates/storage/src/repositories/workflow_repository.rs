use crate::{
    DB,
    model::{WorkflowEntity},
};
use core::{Workflow, WorkerStatus};
use eyre::Result;
use sqlx::{Row, query, query_as};

pub struct WorkflowRepository<'a> {
    db: &'a DB,
}

impl<'a> WorkflowRepository<'a> {
    pub fn new(db: &'a DB) -> Self {
        Self { db }
    }

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
        .execute(self.db)
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
        .fetch_optional(self.db)
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
        .fetch_all(self.db)
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
        
        let total_row = count_query_builder.fetch_one(self.db).await?;
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
        
        let rows: Vec<WorkflowEntity> = data_query_builder.fetch_all(self.db).await?;

        Ok((rows, total as u64))
    }

    /// 检查工作流是否存在
    pub async fn workflow_exists(&self, name: &str) -> Result<bool> {
        let row = query("SELECT COUNT(*) as count FROM workflows WHERE name = ? AND deleted_at IS NULL")
            .bind(name)
            .fetch_one(self.db)
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
        
        query_builder.execute(self.db).await?;
        Ok(())
    }

    /// 删除工作流 (软删除)
    pub async fn delete_workflow(&self, name: &str) -> Result<()> {
        let now = chrono::Utc::now().to_rfc3339();
        query("UPDATE workflows SET deleted_at = ? WHERE name = ?")
            .bind(&now)
            .bind(name)
            .execute(self.db)
            .await?;

        // 同时软删除相关的工作者
        query("UPDATE workers SET deleted_at = ? WHERE workflow_name = ?")
            .bind(&now)
            .bind(name)
            .execute(self.db)
            .await?;

        Ok(())
    }
}
