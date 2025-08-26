use crate::{
    DB,
    model::{TaskEntity, TaskGroupEntity, TaskLogEntity},
};
use core::{Task, TaskGroup, TaskStatus};
use eyre::Result;
use serde_json::json;
use sqlx::{Row, query, query_as};

pub struct Repositories {
    db: DB,
}

impl Repositories {
    pub async fn new(database_url: Option<String>) -> Result<Self> {
        let db = crate::init_database(database_url).await?;
        Ok(Repositories { db })
    }

    // ========== TaskGroup 操作 ==========

    /// 插入任务组
    pub async fn insert_task_group(&self, task_group: &TaskGroup) -> Result<()> {
        query("INSERT INTO task_groups (name, working_dir, status) VALUES (?, ?, ?)")
            .bind(&task_group.name)
            .bind(&task_group.working_dir)
            .bind(&TaskStatus::Created.to_string())
            .execute(&self.db)
            .await?;

        Ok(())
    }

    /// 根据名称获取任务组
    pub async fn get_task_group(&self, name: &str) -> Result<Option<TaskGroupEntity>> {
        let row: Option<TaskGroupEntity> = query_as(
            "SELECT name, working_dir, status, created_at, deleted_at, started_at, completed_at 
             FROM task_groups 
             WHERE name = ? AND deleted_at IS NULL",
        )
        .bind(name)
        .fetch_optional(&self.db)
        .await?;
        Ok(row)
    }

    /// 获取所有任务组
    pub async fn get_all_task_groups(&self) -> Result<Vec<TaskGroupEntity>> {
        let rows: Vec<TaskGroupEntity> = query_as(
            "SELECT name, working_dir, status, created_at, deleted_at, started_at, completed_at 
             FROM task_groups 
             WHERE deleted_at IS NULL",
        )
        .fetch_all(&self.db)
        .await?;

        Ok(rows)
    }

    /// 检查任务组是否存在
    pub async fn task_group_exists(&self, name: &str) -> Result<bool> {
        Ok(
            query("SELECT name FROM task_groups WHERE name = ? AND deleted_at IS NULL")
                .bind(name)
                .fetch_optional(&self.db)
                .await?
                .is_some(),
        )
    }

    /// 更新任务组状态
    pub async fn update_task_group_status(&self, name: &str, status: TaskStatus) -> Result<()> {
        let status_str = status.to_string();

        match status {
            TaskStatus::Created => {
                query("UPDATE task_groups SET status = ?, created_at = ? WHERE name = ?")
                    .bind(&status_str)
                    .bind(chrono::Utc::now().to_rfc3339())
                    .bind(name)
                    .execute(&self.db)
                    .await?;
            }
            TaskStatus::Running => {
                query("UPDATE task_groups SET status = ?, started_at = ? WHERE name = ?")
                    .bind(&status_str)
                    .bind(chrono::Utc::now().to_rfc3339())
                    .bind(name)
                    .execute(&self.db)
                    .await?;
            }
            TaskStatus::Completed | TaskStatus::Failed | TaskStatus::Cancelled => {
                query("UPDATE task_groups SET status = ?, completed_at = ? WHERE name = ?")
                    .bind(&status_str)
                    .bind(chrono::Utc::now().to_rfc3339())
                    .bind(name)
                    .execute(&self.db)
                    .await?;
            }
        }

        Ok(())
    }

    /// 软删除任务组
    pub async fn delete_task_group(&self, name: &str) -> Result<()> {
        let now = chrono::Utc::now().to_rfc3339();

        query("UPDATE task_groups SET deleted_at = ? WHERE name = ?")
            .bind(&now)
            .bind(name)
            .execute(&self.db)
            .await?;

        Ok(())
    }

    // ========== Task 操作 ==========

    /// 批量插入任务
    pub async fn insert_tasks(&self, tasks: &[Task]) -> Result<()> {
        let mut tx = self.db.begin().await?;

        for task in tasks {
            query(
                "INSERT INTO tasks (group_name, name, command, args, working_dir, env_vars, status) 
                 VALUES (?, ?, ?, ?, ?, ?, ?, ?)"
            )
            .bind(&task.group_name)
            .bind(&task.name)
            .bind(&task.command)
            .bind(json!(task.args).to_string())
            .bind(&task.working_dir)
            .bind(json!(task.env_vars).to_string())
            .bind(TaskStatus::Created.to_string())
            .execute(&mut *tx)
            .await?;
        }

        tx.commit().await?;
        Ok(())
    }

    /// 根据组名和任务名获取任务
    pub async fn get_task(&self, group_name: &str, task_name: &str) -> Result<Option<Task>> {
        let row = query(
            "SELECT group_name, name, command, args, working_dir, env_vars, status, error_message,
                    created_at, deleted_at, started_at, completed_at
             FROM tasks 
             WHERE group_name = ? AND name = ? AND deleted_at IS NULL",
        )
        .bind(group_name)
        .bind(task_name)
        .fetch_optional(&self.db)
        .await?;

        if let Some(row) = row {
            let entity = TaskEntity {
                group_name: row.get("group_name"),
                name: row.get("name"),
                command: row.get("command"),
                args: row.get("args"),
                working_dir: row.get("working_dir"),
                env_vars: row.get("env_vars"),
                status: row.get("status"),
                error_message: row.get("error_message"),
                created_at: row.get("created_at"),
                deleted_at: row.get("deleted_at"),
                started_at: row.get("started_at"),
                completed_at: row.get("completed_at"),
            };
            Ok(Some(entity.to_task()?))
        } else {
            Ok(None)
        }
    }

    /// 获取指定组的所有任务
    pub async fn get_tasks_by_group(&self, group_name: &str) -> Result<Vec<TaskEntity>> {
        let rows: Vec<TaskEntity> = query_as(
            "SELECT group_name, name, command, args, working_dir, env_vars, status, error_message,
                    created_at, deleted_at, started_at, completed_at
             FROM tasks 
             WHERE group_name = ? AND deleted_at IS NULL",
        )
        .bind(group_name)
        .fetch_all(&self.db)
        .await?;

        Ok(rows)
    }

    /// 检查任务是否存在
    pub async fn task_exists(&self, group_name: &str, task_name: &str) -> Result<bool> {
        Ok(
            query("SELECT 1 FROM tasks WHERE group_name = ? AND name = ? AND deleted_at IS NULL")
                .bind(group_name)
                .bind(task_name)
                .fetch_optional(&self.db)
                .await?
                .is_some(),
        )
    }

    /// 更新任务状态
    pub async fn update_task_status(
        &self,
        group_name: &str,
        task_name: &str,
        status: TaskStatus,
    ) -> Result<()> {
        let status_str = status.to_string();
        let now = chrono::Utc::now().to_rfc3339();

        query("UPDATE tasks SET status = ?, started_at = ? WHERE group_name = ? AND name = ?")
            .bind(&status_str)
            .bind(&now)
            .bind(group_name)
            .bind(task_name)
            .execute(&self.db)
            .await?;

        Ok(())
    }

    /// 更新任务错误信息
    pub async fn update_task_error(
        &self,
        group_name: &str,
        task_name: &str,
        error_message: &str,
    ) -> Result<()> {
        let status_str = TaskStatus::Failed.to_string();
        let now = chrono::Utc::now().to_rfc3339();

        query(
            "UPDATE tasks SET status = ?, error_message = ?, completed_at = ? 
             WHERE group_name = ? AND name = ?",
        )
        .bind(&status_str)
        .bind(error_message)
        .bind(&now)
        .bind(group_name)
        .bind(task_name)
        .execute(&self.db)
        .await?;

        Ok(())
    }

    /// 软删除任务
    pub async fn delete_task(&self, group_name: &str, task_name: &str) -> Result<()> {
        let now = chrono::Utc::now().to_rfc3339();

        query("UPDATE tasks SET deleted_at = ? WHERE group_name = ? AND name = ?")
            .bind(&now)
            .bind(group_name)
            .bind(task_name)
            .execute(&self.db)
            .await?;

        Ok(())
    }

    // ========== TaskLog 操作 ==========

    /// 插入任务日志记录
    pub async fn insert_task_log(
        &self,
        group_name: &str,
        task_name: &str,
        log_type: &str,
        file_path: &str,
    ) -> Result<()> {
        let now = chrono::Utc::now().to_rfc3339();

        query("INSERT INTO task_logs (group_name, task_name, log_type, file_path, created_at) VALUES (?, ?, ?, ?, ?)")
            .bind(group_name)
            .bind(task_name)
            .bind(log_type)
            .bind(file_path)
            .bind(&now)
            .execute(&self.db)
            .await?;

        Ok(())
    }

    /// 获取任务的日志记录
    pub async fn get_task_logs(
        &self,
        group_name: &str,
        task_name: &str,
    ) -> Result<Vec<TaskLogEntity>> {
        let rows = query(
            "SELECT id, group_name, task_name, log_type, file_path, created_at
             FROM task_logs 
             WHERE group_name = ? AND task_name = ? 
             ORDER BY created_at DESC",
        )
        .bind(group_name)
        .bind(task_name)
        .fetch_all(&self.db)
        .await?;

        let mut logs = Vec::new();
        for row in rows {
            logs.push(TaskLogEntity {
                id: row.get("id"),
                group_name: row.get("group_name"),
                task_name: row.get("task_name"),
                log_type: row.get("log_type"),
                file_path: row.get("file_path"),
                created_at: row.get("created_at"),
            });
        }

        Ok(logs)
    }
}
