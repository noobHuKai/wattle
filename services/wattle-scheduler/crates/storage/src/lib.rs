use std::path::PathBuf;

pub mod model;
mod repositories;

pub use model::{WorkerEntity, WorkflowEntity, WorkerLogEntity};
pub use repositories::Repositories;
pub type DB = sqlx::SqlitePool;

pub async fn init_database(database_url: Option<String>) -> eyre::Result<DB> {
    let db_url = match database_url {
        Some(url) => {
            let db_path = PathBuf::from(&url);
            if db_path.parent().is_some() && !db_path.parent().unwrap().exists() {
                std::fs::create_dir_all(db_path.parent().unwrap())?;
            }
            if !db_path.exists() {
                std::fs::File::create(&db_path)?;
            }
            format!("sqlite:{url}")
        }
        None => "sqlite::memory:".to_string(),
    };
    let pool = sqlx::SqlitePool::connect(&db_url).await?;

    sqlx::migrate!("./migrations").run(&pool).await?;
    Ok(pool)
}
