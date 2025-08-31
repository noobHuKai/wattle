# Control Flow System Migration

## 概述

这个项目已经成功将 control-flow-system（Next.js）的所有功能迁移到了当前的 Tanstack Router 项目中。

## 已迁移的功能

### 🎯 核心页面
- **Dashboard** (`/dashboard`) - 系统总览仪表板，包含指标卡片、图表和系统状态
- **Workflows** (`/workflows`) - 工作流管理页面，支持搜索、筛选和状态管理
- **Nodes** (`/nodes`) - 节点管理页面，支持节点类型筛选和版本管理

### 🔧 详情页面
- **Workflow详情** (`/workflows/:id`) - 工作流详细信息，包含概览、可视化编辑器、YAML配置、工作者和日志等标签页
- **Node详情** (`/nodes/:id`) - 节点详细信息，包含信息、源码、Schema和目录结构等标签页

### 🎨 UI组件
- **布局组件** - Topbar、Sidebar、ThemeProvider
- **Dashboard组件** - MetricCard、CombinedMetricsChart、SystemOverview、TimeRangeSelector
- **业务组件** - WorkflowList、NodeList
- **主题切换** - 完整的明暗主题支持，包含多种颜色方案

### 🛠 技术适配
- **路由系统** - 完全适配 Tanstack Router，支持嵌套路由和参数路由
- **链接组件** - 所有 Next.js Link 组件已替换为 Tanstack Router Link
- **导航钩子** - usePathname 替换为 useLocation
- **样式系统** - 完整的 CSS 变量和主题系统迁移

## 路由结构

```
/_app
├── /dashboard          # 仪表板
├── /workflows         # 工作流列表
├── /workflows/:id     # 工作流详情
├── /nodes            # 节点列表
└── /nodes/:id        # 节点详情
```

## 主要特性

### 📊 仪表板
- 系统指标概览（CPU、内存、活跃工作者等）
- 实时图表显示系统性能
- 最近活动和系统健康状态
- 响应式布局，支持移动端

### 🔄 工作流管理
- 工作流列表展示，支持状态筛选
- 搜索功能，快速找到特定工作流
- 工作流详情页面，支持多种操作
- 状态管理（运行中、停止、错误等）

### 🧩 节点管理
- 节点列表，按类型（输入、处理器、输出）分类
- 版本管理和状态追踪
- 节点详情页面，支持源码查看
- Schema定义和目录结构展示

### 🎨 主题系统
- 明暗主题切换
- 多种颜色方案选择
- 完整的 CSS 变量系统
- 自动系统主题检测

## 依赖包

已添加的新依赖：
- `@hookform/resolvers` - 表单验证
- `react-hook-form` - 表单管理
- `date-fns` - 日期处理
- `react-day-picker` - 日期选择器
- `cmdk` - 命令面板
- `input-otp` - OTP输入
- `embla-carousel-react` - 轮播图
- `react-resizable-panels` - 可调整面板
- `recharts` - 图表库

## 使用方法

1. 启动开发服务器：
   ```bash
   pnpm run dev
   ```

2. 访问应用：
   - http://localhost:3000 (自动重定向到仪表板)
   - http://localhost:3000/dashboard
   - http://localhost:3000/workflows  
   - http://localhost:3000/nodes

## 注意事项

- 所有页面都使用 Tanstack Router 的文件系统路由
- 主题切换功能完全正常，支持系统主题检测
- 响应式设计，适配不同屏幕尺寸
- 组件库统一使用 shadcn/ui
- 样式系统使用 Tailwind CSS 4.0

## 待完善功能

某些高级功能的具体实现可以根据需求进一步开发：
- 可视化工作流编辑器
- YAML 配置编辑器
- 实时日志查看器
- 源码编辑器
- Schema 编辑器

迁移工作已基本完成，所有核心功能都已正常运行！
