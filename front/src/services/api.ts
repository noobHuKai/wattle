import { 
  generateMockMonitorData,
  generateMockTasks,
  generateMockTaskGroups,
  generateMockTaskGroupDetail,
  generateMockTaskLogs
} from '@/data/mock-data';

export interface MonitorData {
  timestamp: string;
  cpu: number;
  memory: number; // MB
}

export interface Task {
  id?: string;
  name: string;
  groupName: string; // group_name in API
  status: 'created' | 'running' | 'completed' | 'failed' | 'cancelled';
  createTime: string; // created_at in API
  command?: string;
  startedAt?: string | null; // started_at in API
  completedAt?: string | null; // completed_at in API
  exitCode?: number | null; // exit_code in API
}

export interface TaskGroup {
  id?: string;
  name: string;
  status: 'created' | 'running' | 'completed' | 'failed' | 'cancelled' | 'active' | 'inactive';
  createTime: string; // created_at in API
  taskCount?: number; // task_count in API
  description?: string;
  maxConcurrency?: number | null; // max_concurrency in API
  executionStrategy?: string; // execution_strategy in API
  timeout?: number | null;
  startedAt?: string | null; // started_at in API
  completedAt?: string | null; // completed_at in API
  runningTasks?: number; // running_tasks in API
  completedTasks?: number; // completed_tasks in API
}

export interface TaskGroupDetail extends TaskGroup {
  tasks: Task[];
  topics: string[];
}

// API调用函数
export const api = {
  // Dashboard API
  getMonitorData: async (_timeRange?: { start: string; end: string }): Promise<MonitorData[]> => {
    console.warn('强制使用模拟数据: getMonitorData');
    await new Promise(resolve => setTimeout(resolve, 500)); // 模拟网络延迟
    return generateMockMonitorData();
  },

  getCurrentMonitorData: async (): Promise<MonitorData> => {
    console.warn('强制使用模拟数据: getCurrentMonitorData');
    await new Promise(resolve => setTimeout(resolve, 100)); // 模拟网络延迟
    return {
      timestamp: new Date().toISOString(),
      cpu: Math.random() * 80 + 10,
      memory: Math.random() * 6000 + 2000
    };
  },

  // Task API
  getTasks: async (filters?: { name?: string; status?: string; groupName?: string }): Promise<Task[]> => {
    console.warn('强制使用模拟数据: getTasks');
    await new Promise(resolve => setTimeout(resolve, 500)); // 模拟网络延迟
    let tasks = generateMockTasks();
    
    // 应用过滤条件
    if (filters?.name) {
      tasks = tasks.filter(task => 
        task.name.toLowerCase().includes(filters.name!.toLowerCase())
      );
    }
    if (filters?.status) {
      tasks = tasks.filter(task => task.status === filters.status);
    }
    if (filters?.groupName) {
      tasks = tasks.filter(task => 
        task.groupName.toLowerCase().includes(filters.groupName!.toLowerCase())
      );
    }
    
    return tasks;
  },

  getTaskLogs: async (taskId: string): Promise<string> => {
    console.warn('强制使用模拟数据: getTaskLogs');
    await new Promise(resolve => setTimeout(resolve, 300)); // 模拟网络延迟
    return generateMockTaskLogs(taskId);
  },

  // Task Group API
  getTaskGroups: async (filters?: { name?: string; status?: string }): Promise<TaskGroup[]> => {
    console.warn('强制使用模拟数据: getTaskGroups');
    await new Promise(resolve => setTimeout(resolve, 500)); // 模拟网络延迟
    let groups = generateMockTaskGroups();
    
    // 应用过滤条件
    if (filters?.name) {
      groups = groups.filter(group => 
        group.name.toLowerCase().includes(filters.name!.toLowerCase())
      );
    }
    if (filters?.status) {
      groups = groups.filter(group => group.status === filters.status);
    }
    
    return groups;
  },

  getTaskGroupDetail: async (groupId: string): Promise<TaskGroupDetail> => {
    console.warn('强制使用模拟数据: getTaskGroupDetail');
    await new Promise(resolve => setTimeout(resolve, 500)); // 模拟网络延迟
    return generateMockTaskGroupDetail(groupId);
  },

  createTaskGroup: async (data: { name: string }): Promise<TaskGroup> => {
    console.warn('强制使用模拟数据: createTaskGroup');
    await new Promise(resolve => setTimeout(resolve, 500)); // 模拟网络延迟
    // 模拟创建成功
    return {
      id: `group-${Date.now()}`,
      name: data.name,
      status: 'created',
      createTime: new Date().toISOString(),
      taskCount: 0,
      description: `模拟创建的任务组: ${data.name}`,
      runningTasks: 0,
      completedTasks: 0,
    };
  },
};
