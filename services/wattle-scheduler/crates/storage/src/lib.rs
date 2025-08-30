use core::DatabaseConfig;
use sqlx::sqlite::SqlitePoolOptions;
use std::path::PathBuf;

pub mod model;
mod repositories;

pub use model::{WorkerEntity, WorkflowEntity, WorkerLogEntity};
pub use repositories::{Repositories, WorkflowRepository, WorkerRepository, LogRepository};
pub type DB = sqlx::SqlitePool;

pub async fn init_database(
    database_url: Option<String>,
    db_config: Option<DatabaseConfig>,
) -> eyre::Result<DB> {
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

    let mut pool_options = SqlitePoolOptions::new();
    if let Some(config) = db_config {
        if let Some(max_conn) = config.max_connections {
            pool_options = pool_options.max_connections(max_conn);
        }
    }

    let pool = pool_options.connect(&db_url).await?;

    sqlx::migrate!("./migrations").run(&pool).await?;
    Ok(pool)
}
