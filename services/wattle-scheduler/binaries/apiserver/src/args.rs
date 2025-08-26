//! 命令行参数
//!

use std::path::PathBuf;

use clap::Parser;

/// 命令行参数
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
pub struct Args {
    #[arg(short, long, default_value = "configs/config.toml")]
    pub config: PathBuf,
}
