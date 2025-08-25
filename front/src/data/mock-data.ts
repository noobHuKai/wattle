import type { Task, TaskGroup, MonitorData, TaskGroupDetail } from '@/services/api';

// 生成监控数据
export const generateMockMonitorData = (): MonitorData[] => {
  const data: MonitorData[] = [];
  const now = new Date();
  for (let i = 59; i >= 0; i--) {
    const time = new Date(now.getTime() - i * 5 * 60 * 1000);
    data.push({
      timestamp: time.toISOString(),
      cpu: Math.random() * 80 + 10,
      memory: Math.random() * 6000 + 2000
    });
  }
  return data;
};

// 生成任务数据
export const generateMockTasks = (): Task[] => {
  const groups = ['数据处理组', '机器学习组', '前端开发组', '后端服务组', 'DevOps运维组', '测试自动化组'];
  const statuses: Task['status'][] = ['created', 'running', 'completed', 'failed', 'cancelled'];
  const commands = [
    'python data_processor.py --input data.csv --output result.json',
    'npm run build && npm run deploy',
    'docker build -t myapp:latest . && docker push myapp:latest',
    'python train_model.py --epochs 100 --batch-size 32',
    'node server.js --port 3000',
    'python -m pytest tests/ --coverage',
    'yarn install && yarn start',
    'mvn clean package -DskipTests',
    'kubectl apply -f deployment.yaml',
    'terraform plan && terraform apply',
    'ansible-playbook -i inventory playbook.yml',
    'jupyter notebook --no-browser --port 8888'
  ];
  
  const taskNames = [
    '数据清洗任务',
    '模型训练任务', 
    '前端构建任务',
    'API测试任务',
    'Docker部署任务',
    '数据库迁移任务',
    '性能测试任务',
    '代码质量检查',
    '安全扫描任务',
    '备份数据任务',
    '日志分析任务',
    '监控配置任务'
  ];
  
  const tasks: Task[] = [];
  
  for (let i = 1; i <= 30; i++) {
    const now = new Date();
    const createTime = new Date(now.getTime() - Math.random() * 7 * 24 * 60 * 60 * 1000);
    const status = statuses[Math.floor(Math.random() * statuses.length)];
    
    // 根据状态设置时间
    let startedAt: string | null = null;
    let completedAt: string | null = null;
    let exitCode: number | null = null;
    
    if (status !== 'created') {
      startedAt = new Date(createTime.getTime() + Math.random() * 60 * 60 * 1000).toISOString();
      
      if (status === 'completed') {
        completedAt = new Date(new Date(startedAt).getTime() + Math.random() * 30 * 60 * 1000).toISOString();
        exitCode = 0;
      } else if (status === 'failed') {
        completedAt = new Date(new Date(startedAt).getTime() + Math.random() * 20 * 60 * 1000).toISOString();
        exitCode = Math.floor(Math.random() * 10) + 1;
      } else if (status === 'cancelled') {
        completedAt = new Date(new Date(startedAt).getTime() + Math.random() * 15 * 60 * 1000).toISOString();
        exitCode = 130; // 取消信号
      }
    }
    
    const taskName = taskNames[Math.floor(Math.random() * taskNames.length)];
    
    tasks.push({
      id: `task-${String(i).padStart(3, '0')}`,
      name: `${taskName}-${String(i).padStart(3, '0')}`,
      groupName: groups[Math.floor(Math.random() * groups.length)],
      status,
      createTime: createTime.toISOString(),
      command: commands[Math.floor(Math.random() * commands.length)],
      startedAt,
      completedAt,
      exitCode,
    });
  }
  return tasks;
};

// 生成任务组数据
export const generateMockTaskGroups = (): TaskGroup[] => {
  const groups = [
    { 
      id: 'group-1', 
      name: '数据处理组', 
      status: 'active' as const,
      description: '负责处理和分析各种数据源的任务组，包括ETL、数据清洗和数据转换任务',
      executionStrategy: 'sequential'
    },
    { 
      id: 'group-2', 
      name: '机器学习组', 
      status: 'running' as const,
      description: '机器学习模型训练和推理任务，包括深度学习和传统ML算法',
      executionStrategy: 'parallel'
    },
    { 
      id: 'group-3', 
      name: '前端开发组', 
      status: 'inactive' as const,
      description: '前端应用构建和部署任务，包括React、Vue等框架的构建',
      executionStrategy: 'sequential'
    },
    { 
      id: 'group-4', 
      name: '后端服务组', 
      status: 'completed' as const,
      description: '后端服务开发和测试任务，包括API开发、微服务部署等',
      executionStrategy: 'parallel'
    },
    { 
      id: 'group-5', 
      name: 'DevOps运维组', 
      status: 'active' as const,
      description: 'CI/CD和基础设施管理任务，包括容器化、K8s部署等',
      executionStrategy: 'sequential'
    },
    { 
      id: 'group-6', 
      name: '测试自动化组', 
      status: 'failed' as const,
      description: '自动化测试和质量保证任务，包括单元测试、集成测试、性能测试',
      executionStrategy: 'parallel'
    },
    { 
      id: 'group-7', 
      name: '数据库维护组', 
      status: 'active' as const,
      description: '数据库备份、迁移和优化任务',
      executionStrategy: 'sequential'
    },
    { 
      id: 'group-8', 
      name: '安全审计组', 
      status: 'completed' as const,
      description: '安全扫描、漏洞检测和合规性检查任务',
      executionStrategy: 'parallel'
    },
  ];
  
  return groups.map(group => {
    const now = new Date();
    const createTime = new Date(now.getTime() - Math.random() * 30 * 24 * 60 * 60 * 1000);
    const taskCount = Math.floor(Math.random() * 15) + 3;
    const runningTasks = group.status === 'running' || group.status === 'active' 
      ? Math.floor(Math.random() * Math.min(taskCount, 5)) 
      : 0;
    const completedTasks = group.status === 'completed' 
      ? taskCount 
      : Math.floor(Math.random() * (taskCount - runningTasks));
    
    // 根据状态设置时间
    let startedAt: string | null = null;
    let completedAt: string | null = null;
    
    if (group.status !== 'inactive') {
      startedAt = new Date(createTime.getTime() + Math.random() * 24 * 60 * 60 * 1000).toISOString();
      
      if (group.status === 'completed') {
        completedAt = new Date(new Date(startedAt).getTime() + Math.random() * 7 * 24 * 60 * 60 * 1000).toISOString();
      }
    }
    
    return {
      id: group.id,
      name: group.name,
      status: group.status,
      createTime: createTime.toISOString(),
      taskCount,
      description: group.description,
      maxConcurrency: Math.floor(Math.random() * 5) + 1,
      executionStrategy: group.executionStrategy,
      timeout: Math.floor(Math.random() * 3600) + 300, // 5分钟到1小时
      startedAt,
      completedAt,
      runningTasks,
      completedTasks,
    };
  });
};

// 生成任务组详情数据
export const generateMockTaskGroupDetail = (groupId: string): TaskGroupDetail => {
  const groups = generateMockTaskGroups();
  const group = groups.find(g => g.id === groupId);
  
  if (!group) {
    // 如果找不到组，返回一个默认的模拟组详情
    const mockGroup = groups[0] || {
      id: 'group-fallback',
      name: '备用数据处理组',
      status: 'active' as const,
      createTime: new Date().toISOString(),
      taskCount: 0,
      description: '默认的备用任务组',
      maxConcurrency: 3,
      executionStrategy: 'sequential',
      timeout: 1800,
      runningTasks: 0,
      completedTasks: 0,
    };
    
    return {
      ...mockGroup,
      id: groupId,
      name: `未找到的组 (${groupId})`,
      tasks: [],
      topics: ['模拟数据', '错误处理'],
    };
  }
  
  const allTasks = generateMockTasks();
  const groupTasks = allTasks.filter(task => 
    task.groupName === group.name
  );
  
  // 生成相关主题
  const allTopics = [
    '数据处理', '机器学习', 'ETL', '模型训练', 'API开发',
    '前端构建', '后端服务', 'DevOps', 'CI/CD', '测试自动化',
    '数据库维护', '安全审计', '性能优化', '监控告警', '日志分析'
  ];
  
  const topics = allTopics
    .sort(() => Math.random() - 0.5)
    .slice(0, Math.floor(Math.random() * 5) + 3);
  
  return {
    ...group,
    tasks: groupTasks,
    topics,
  };
};

// 生成任务日志
export const generateMockTaskLogs = (taskId: string): string => {
  const logEntries = [
    '任务开始执行',
    '初始化环境变量',
    '检查依赖项',
    '加载配置文件',
    '连接数据库',
    '开始处理数据',
    '执行主要业务逻辑',
    '处理中间结果',
    '验证输出结果',
    '清理临时文件',
    '任务执行完成'
  ];
  
  const now = new Date();
  const logs = logEntries.map((entry, index) => {
    const timestamp = new Date(now.getTime() - (logEntries.length - index) * 1000);
    const level = Math.random() > 0.8 ? 'WARN' : (Math.random() > 0.1 ? 'INFO' : 'ERROR');
    return `${timestamp.toISOString().replace('T', ' ').substring(0, 19)} ${level.padEnd(5)} ${entry}`;
  }).join('\n');
  
  return `任务日志 - ${taskId}\n\n${logs}\n\n注意：这是模拟日志数据，用于演示目的`;
};
