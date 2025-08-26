use clap::Parser as _;
use tracing_error::ErrorLayer;
use tracing_subscriber::{EnvFilter, fmt, prelude::*};
mod args;
mod config;
mod server;

pub use config::Config;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    dotenv::dotenv().ok();
    // Init Tracing
    let fmt_layer = fmt::layer().with_target(false);
    let filter_layer = EnvFilter::try_from_default_env()
        .or_else(|_| EnvFilter::try_new("info"))
        .unwrap();

    tracing_subscriber::registry()
        .with(filter_layer)
        .with(fmt_layer)
        .with(ErrorLayer::default())
        .init();

    // Init color_eyre
    color_eyre::config::HookBuilder::default()
        .add_frame_filter(Box::new(|frames| {
            let filters = &["custom_filter::main"];

            frames.retain(|frame| {
                !filters.iter().any(|f| {
                    let name = if let Some(name) = frame.name.as_ref() {
                        name.as_str()
                    } else {
                        return true;
                    };

                    name.starts_with(f)
                })
            });
        }))
        .install()
        .unwrap();

    // Init Command Line Args
    let args = args::Args::parse();

    // Run Daemon
    if let Err(err) = run_daemon(args).await {
        tracing::error!("{}", err);
    };

    Ok(())
}

pub async fn run_daemon(args: args::Args) -> eyre::Result<()> {
    // Parse Config File
    let cfg = config::Config::new(args.config)?;
    // Run Server
    server::run_server(cfg).await?;
    Ok(())
}
