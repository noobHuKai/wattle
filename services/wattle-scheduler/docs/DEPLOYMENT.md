# Wattle Scheduler 部署和使用指南

## 快速开始

### 系统要求

#### 操作系统支持
- Linux (推荐 Ubuntu 20.04+, CentOS 8+)
- macOS (10.15+)
- Windows (10+)

#### 软件依赖
- **Rust**: 1.70.0 或更高版本
- **Git**: 用于代码管理
- **SQLite**: 数据库（自动包含）
- **Zenoh**: 分布式通信（自动管理）

### 安装步骤

#### 1. 安装 Rust 工具链
```bash
# 安装 Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

# 验证安装
rustc --version
cargo --version
```

#### 2. 克隆项目
```bash
git clone <repository-url>
cd wattle/services/wattle-scheduler
```

#### 3. 编译项目
```bash
# 编译调试版本
cargo build

# 编译发布版本（推荐生产环境）
cargo build --release
```

#### 4. 运行测试
```bash
# 运行所有测试
cargo test

# 运行特定模块测试
cargo test --package coordinator
cargo test --package storage
```

## 配置文件

### 基础配置 (configs/config.toml)
```toml
[server]
# API 服务器配置
host = "0.0.0.0"
port = 9240

[coordinator]
# 执行模式: Sequential, Parallel, DependencyBased
mode = "DependencyBased"
# 数据库文件路径
db_url = "configs/wattle.db"

[execution]
# 最大并发工作者数量
max_concurrent_workers = 10
# 日志目录
log_directory = "logs/"
# 工作者超时时间（秒）
worker_timeout = 300
```

### 环境变量配置
```bash
# 数据库配置
export WATTLE_DB_URL="sqlite:///path/to/wattle.db"

# 日志级别
export RUST_LOG="wattle=info,coordinator=debug"

# API 服务器配置
export WATTLE_HOST="0.0.0.0"
export WATTLE_PORT="9240"

# Zenoh 配置
export ZENOH_ROUTER_PORT="7447"
```

## 启动服务

### 1. 启动 API 服务器
```bash
# 使用默认配置
./target/release/apiserver

# 使用自定义配置
./target/release/apiserver --config /path/to/config.toml

# 后台运行
nohup ./target/release/apiserver --config configs/config.toml > apiserver.log 2>&1 &
```

### 2. 验证服务状态
```bash
# 检查服务健康状态
curl http://localhost:9240/api/workflows

# 查看系统日志
tail -f apiserver.log
```

## CLI 工具使用

### 基本命令
```bash
# 列出所有工作流
./target/release/cli list

# 生成示例工作流
./target/release/cli example

# 创建工作流
./target/release/cli create --input workflow.json

# 启动工作流
./target/release/cli run my-workflow

# 查看工作流状态
./target/release/cli status my-workflow
```

### 工作流定义文件示例 (workflow.json)
```json
{
  "name": "example-pipeline",
  "working_dir": "/tmp/example-pipeline",
  "workers": [
    {
      "name": "data_generator",
      "workflow_name": "example-pipeline",
      "command": "python3",
      "args": ["-c", "import json; import random; data = [random.randint(1, 100) for _ in range(10)]; print(json.dumps({'numbers': data}))"],
      "working_dir": "/tmp/example-pipeline",
      "env_vars": {
        "PYTHONUNBUFFERED": "1"
      },
      "outputs": {
        "numbers": "/tmp/example-pipeline/numbers.json"
      }
    },
    {
      "name": "data_analyzer",
      "workflow_name": "example-pipeline",
      "command": "python3",
      "args": ["-c", "import json; data = json.load(open('/tmp/example-pipeline/numbers.json')); result = {'sum': sum(data['numbers']), 'avg': sum(data['numbers'])/len(data['numbers']), 'max': max(data['numbers']), 'min': min(data['numbers'])}; print(json.dumps(result))"],
      "working_dir": "/tmp/example-pipeline",
      "env_vars": {
        "PYTHONUNBUFFERED": "1"
      },
      "inputs": {
        "input_data": "data_generator/numbers"
      },
      "outputs": {
        "analysis": "/tmp/example-pipeline/analysis.json"
      }
    },
    {
      "name": "report_generator",
      "workflow_name": "example-pipeline",
      "command": "python3",
      "args": ["-c", "import json; analysis = json.load(open('/tmp/example-pipeline/analysis.json')); report = f'Analysis Report:\\nSum: {analysis[\"sum\"]}\\nAverage: {analysis[\"avg\"]:.2f}\\nMax: {analysis[\"max\"]}\\nMin: {analysis[\"min\"]}'; print(report); open('/tmp/example-pipeline/report.txt', 'w').write(report)"],
      "working_dir": "/tmp/example-pipeline",
      "env_vars": {
        "PYTHONUNBUFFERED": "1"
      },
      "inputs": {
        "analysis_data": "data_analyzer/analysis"
      },
      "outputs": {
        "report": "/tmp/example-pipeline/report.txt"
      }
    }
  ]
}
```

## Docker 部署

### 1. 创建 Dockerfile
```dockerfile
# Dockerfile
FROM rust:1.75 as builder

WORKDIR /app
COPY . .
RUN cargo build --release

FROM debian:bullseye-slim

RUN apt-get update && apt-get install -y \
    ca-certificates \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=builder /app/target/release/apiserver /usr/local/bin/
COPY --from=builder /app/target/release/cli /usr/local/bin/
COPY configs/ /app/configs/

EXPOSE 9240

CMD ["apiserver", "--config", "/app/configs/config.toml"]
```

### 2. 构建和运行容器
```bash
# 构建镜像
docker build -t wattle-scheduler .

# 运行容器
docker run -d \
  --name wattle-scheduler \
  -p 9240:9240 \
  -v $(pwd)/logs:/app/logs \
  -v $(pwd)/data:/app/data \
  wattle-scheduler

# 查看容器状态
docker ps
docker logs wattle-scheduler
```

### 3. Docker Compose 部署
```yaml
# docker-compose.yml
version: '3.8'

services:
  wattle-scheduler:
    build: .
    ports:
      - "9240:9240"
    volumes:
      - ./logs:/app/logs
      - ./data:/app/data
      - ./configs:/app/configs
    environment:
      - RUST_LOG=info
      - WATTLE_HOST=0.0.0.0
      - WATTLE_PORT=9240
    restart: unless-stopped

  wattle-frontend:
    build: ../../../front
    ports:
      - "3000:3000"
    environment:
      - VITE_API_URL=http://localhost:9240
    depends_on:
      - wattle-scheduler
    restart: unless-stopped
```

```bash
# 启动所有服务
docker-compose up -d

# 查看服务状态
docker-compose ps

# 查看服务日志
docker-compose logs -f wattle-scheduler
```

## 生产环境部署

### 1. 系统服务配置 (systemd)
```ini
# /etc/systemd/system/wattle-scheduler.service
[Unit]
Description=Wattle Scheduler Service
After=network.target

[Service]
Type=simple
User=wattle
Group=wattle
WorkingDirectory=/opt/wattle-scheduler
ExecStart=/opt/wattle-scheduler/target/release/apiserver --config /opt/wattle-scheduler/configs/config.toml
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=wattle-scheduler

[Install]
WantedBy=multi-user.target
```

```bash
# 创建用户
sudo useradd -r -s /bin/false wattle

# 设置权限
sudo chown -R wattle:wattle /opt/wattle-scheduler

# 启用和启动服务
sudo systemctl enable wattle-scheduler
sudo systemctl start wattle-scheduler
sudo systemctl status wattle-scheduler
```

### 2. 反向代理配置 (Nginx)
```nginx
# /etc/nginx/sites-available/wattle-scheduler
server {
    listen 80;
    server_name wattle.yourdomain.com;

    location / {
        proxy_pass http://127.0.0.1:9240;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        
        # SSE 支持
        proxy_buffering off;
        proxy_cache off;
        proxy_set_header Connection '';
        proxy_http_version 1.1;
        chunked_transfer_encoding off;
    }

    location /api/workers/*/logs/stream {
        proxy_pass http://127.0.0.1:9240;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        
        # SSE 特殊配置
        proxy_buffering off;
        proxy_cache off;
        proxy_set_header Connection '';
        proxy_http_version 1.1;
        proxy_read_timeout 86400;
    }
}
```

```bash
# 启用站点
sudo ln -s /etc/nginx/sites-available/wattle-scheduler /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl reload nginx
```

### 3. SSL/TLS 配置 (Let's Encrypt)
```bash
# 安装 Certbot
sudo apt-get install certbot python3-certbot-nginx

# 获取 SSL 证书
sudo certbot --nginx -d wattle.yourdomain.com

# 自动续期
sudo crontab -e
# 添加: 0 12 * * * /usr/bin/certbot renew --quiet
```

## 监控和日志

### 1. 日志配置
```bash
# 设置日志级别
export RUST_LOG="info,wattle_scheduler=debug,coordinator=debug"

# 日志文件轮转 (logrotate)
sudo tee /etc/logrotate.d/wattle-scheduler << EOF
/opt/wattle-scheduler/logs/*.log {
    daily
    missingok
    rotate 52
    compress
    notifempty
    create 644 wattle wattle
    postrotate
        systemctl reload wattle-scheduler
    endscript
}
EOF
```

### 2. 系统监控脚本
```bash
#!/bin/bash
# monitor.sh - 健康检查脚本

API_URL="http://localhost:9240"
LOG_FILE="/var/log/wattle-monitor.log"

check_health() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    if curl -s -f "${API_URL}/api/workflows" > /dev/null; then
        echo "[$timestamp] Service is healthy" >> $LOG_FILE
        return 0
    else
        echo "[$timestamp] Service is unhealthy" >> $LOG_FILE
        return 1
    fi
}

restart_service() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] Restarting wattle-scheduler service" >> $LOG_FILE
    sudo systemctl restart wattle-scheduler
}

# 主检查逻辑
if ! check_health; then
    sleep 30
    if ! check_health; then
        restart_service
    fi
fi
```

```bash
# 设置定时检查 (cron)
# 每5分钟检查一次
*/5 * * * * /opt/wattle-scheduler/monitor.sh
```

### 3. Prometheus 集成
```toml
# 在 config.toml 中添加
[metrics]
enabled = true
endpoint = "/metrics"
```

```yaml
# prometheus.yml 配置
scrape_configs:
  - job_name: 'wattle-scheduler'
    static_configs:
      - targets: ['localhost:9240']
    metrics_path: '/metrics'
    scrape_interval: 30s
```

## 性能优化

### 1. 系统级优化
```bash
# 增加文件描述符限制
echo "wattle soft nofile 65536" >> /etc/security/limits.conf
echo "wattle hard nofile 65536" >> /etc/security/limits.conf

# 优化 TCP 参数
echo "net.core.somaxconn = 65536" >> /etc/sysctl.conf
echo "net.core.netdev_max_backlog = 5000" >> /etc/sysctl.conf
sysctl -p
```

### 2. 应用级优化
```toml
# config.toml 性能调优
[coordinator]
mode = "DependencyBased"
db_url = "configs/wattle.db"

[execution]
max_concurrent_workers = 20  # 根据 CPU 核心数调整
worker_timeout = 600         # 增加超时时间
log_buffer_size = 8192       # 日志缓冲区大小

[database]
connection_pool_size = 10    # 数据库连接池大小
max_idle_connections = 5     # 最大空闲连接数
```

### 3. 数据库优化
```sql
-- 创建索引优化查询
CREATE INDEX idx_workflows_status ON workflows(status);
CREATE INDEX idx_workers_status ON workers(status);
CREATE INDEX idx_workers_workflow ON workers(workflow_name);

-- 定期清理过期数据
DELETE FROM workers WHERE deleted_at IS NOT NULL AND deleted_at < datetime('now', '-30 days');
DELETE FROM workflows WHERE deleted_at IS NOT NULL AND deleted_at < datetime('now', '-30 days');
```

## 故障排除

### 1. 常见问题

#### 服务启动失败
```bash
# 检查端口占用
netstat -tlnp | grep 9240
lsof -i :9240

# 检查配置文件
./target/release/apiserver --config configs/config.toml --check-config

# 查看详细错误日志
RUST_LOG=debug ./target/release/apiserver --config configs/config.toml
```

#### Zenoh 连接问题
```bash
# 检查 Zenoh 进程
ps aux | grep zenohd

# 手动启动 Zenoh
zenohd --rest-http-port 8000

# 测试 Zenoh 连接
curl http://127.0.0.1:8000/@/router/version
```

#### 数据库问题
```bash
# 检查数据库文件权限
ls -la configs/wattle.db

# 手动运行迁移
sqlx migrate run --database-url sqlite:configs/wattle.db

# 重置数据库
rm configs/wattle.db
./target/release/apiserver --config configs/config.toml --reset-db
```

### 2. 调试技巧

#### 启用详细日志
```bash
export RUST_LOG="debug,sqlx=debug,hyper=debug"
export RUST_BACKTRACE=full
./target/release/apiserver --config configs/config.toml
```

#### 使用 strace 追踪系统调用
```bash
strace -p $(pidof apiserver) -o trace.log
```

#### 性能分析
```bash
# 使用 perf 分析性能热点
perf record -g ./target/release/apiserver --config configs/config.toml
perf report
```

### 3. 紧急恢复

#### 数据库备份恢复
```bash
# 创建备份
cp configs/wattle.db configs/wattle.db.backup

# 恢复备份
cp configs/wattle.db.backup configs/wattle.db
```

#### 服务重启
```bash
# 优雅重启
sudo systemctl reload wattle-scheduler

# 强制重启
sudo systemctl restart wattle-scheduler

# 紧急停止
sudo systemctl kill wattle-scheduler
```

## 安全配置

### 1. 防火墙设置
```bash
# UFW 防火墙配置
sudo ufw allow 9240/tcp
sudo ufw allow from 10.0.0.0/8 to any port 9240
sudo ufw enable
```

### 2. 用户权限
```bash
# 创建专用用户
sudo useradd -r -s /bin/false -d /opt/wattle-scheduler wattle

# 设置文件权限
sudo chown -R wattle:wattle /opt/wattle-scheduler
sudo chmod 750 /opt/wattle-scheduler
sudo chmod 640 /opt/wattle-scheduler/configs/config.toml
```

### 3. 网络安全
```bash
# 绑定到本地接口
[server]
host = "127.0.0.1"  # 仅本地访问
port = 9240

# 使用反向代理提供外部访问
# 在 Nginx 中配置 SSL 和访问控制
```

## 升级指南

### 1. 版本升级步骤
```bash
# 1. 备份数据
cp -r /opt/wattle-scheduler /opt/wattle-scheduler.backup

# 2. 停止服务
sudo systemctl stop wattle-scheduler

# 3. 更新代码
cd /opt/wattle-scheduler
git pull origin main
cargo build --release

# 4. 运行数据库迁移
sqlx migrate run --database-url sqlite:configs/wattle.db

# 5. 启动服务
sudo systemctl start wattle-scheduler

# 6. 验证升级
curl http://localhost:9240/api/workflows
```

### 2. 回滚步骤
```bash
# 1. 停止服务
sudo systemctl stop wattle-scheduler

# 2. 恢复备份
rm -rf /opt/wattle-scheduler
mv /opt/wattle-scheduler.backup /opt/wattle-scheduler

# 3. 启动服务
sudo systemctl start wattle-scheduler
```

这个部署和使用指南涵盖了从基础安装到生产环境部署的完整流程，包括性能优化、监控、故障排除和安全配置等关键内容。用户可以根据自己的需求选择合适的部署方式。
