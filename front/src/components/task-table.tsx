import { useState } from "react";
import { useQuery } from "@tanstack/react-query";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Dialog, DialogContent, DialogDescription, DialogHeader, DialogTitle, DialogTrigger } from "@/components/ui/dialog";
import { ScrollArea } from "@/components/ui/scroll-area";
import { RefreshCw } from "lucide-react";
import { api } from "@/services/api";
import { formatTimeAgo } from "@/utils/time";
import type { Task } from "@/services/api";

interface TaskTableProps {
  data: Task[];
  isLoading: boolean;
  onRefresh: () => void;
}

const getStatusBadgeVariant = (status: Task['status']) => {
  switch (status) {
    case 'created':
      return 'secondary';
    case 'running':
      return 'default';
    case 'completed':
      return 'outline';
    case 'failed':
      return 'destructive';
    case 'cancelled':
      return 'secondary';
    default:
      return 'secondary';
  }
};

const getStatusLabel = (status: Task['status']) => {
  switch (status) {
    case 'created':
      return '已创建';
    case 'running':
      return '运行中';
    case 'completed':
      return '已完成';
    case 'failed':
      return '失败';
    case 'cancelled':
      return '已取消';
    default:
      return status;
  }
};

function TaskLogsDialog({ taskId, taskName }: { taskId: string; taskName: string }) {
  const [isOpen, setIsOpen] = useState(false);
  
  const { data: logs, isLoading, refetch } = useQuery({
    queryKey: ['task-logs', taskId],
    queryFn: () => api.getTaskLogs(taskId),
    enabled: isOpen, // 只在对话框打开时查询
  });

  const handleOpenChange = (open: boolean) => {
    setIsOpen(open);
    if (open) {
      refetch();
    }
  };

  return (
    <Dialog open={isOpen} onOpenChange={handleOpenChange}>
      <DialogTrigger asChild>
        <Button variant="outline" size="sm">
          查看日志
        </Button>
      </DialogTrigger>
      <DialogContent className="max-w-4xl max-h-[80vh]">
        <DialogHeader>
          <DialogTitle>任务日志 - {taskName}</DialogTitle>
          <DialogDescription>任务ID: {taskId}</DialogDescription>
        </DialogHeader>
        <ScrollArea className="h-[60vh] w-full">
          {isLoading ? (
            <div className="flex items-center justify-center h-32">
              <div className="text-muted-foreground">加载日志中...</div>
            </div>
          ) : (
            <pre className="text-sm whitespace-pre-wrap font-mono bg-muted p-4 rounded">
              {logs || '暂无日志'}
            </pre>
          )}
        </ScrollArea>
      </DialogContent>
    </Dialog>
  );
}

export function TaskTable({ data, isLoading, onRefresh }: TaskTableProps) {
  const [sortField, setSortField] = useState<keyof Task>('createTime');
  const [sortOrder, setSortOrder] = useState<'asc' | 'desc'>('desc');

  const sortedData = [...data].sort((a, b) => {
    const aValue = a[sortField];
    const bValue = b[sortField];
    
    if (sortField === 'createTime') {
      const aDate = new Date(aValue as string).getTime();
      const bDate = new Date(bValue as string).getTime();
      return sortOrder === 'asc' ? aDate - bDate : bDate - aDate;
    }
    
    if (typeof aValue === 'string' && typeof bValue === 'string') {
      return sortOrder === 'asc' 
        ? aValue.localeCompare(bValue) 
        : bValue.localeCompare(aValue);
    }
    
    return 0;
  });

  const handleSort = (field: keyof Task) => {
    if (sortField === field) {
      setSortOrder(sortOrder === 'asc' ? 'desc' : 'asc');
    } else {
      setSortField(field);
      setSortOrder('desc');
    }
  };

  if (isLoading) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>任务列表</CardTitle>
          <CardDescription>所有任务的状态和详情</CardDescription>
        </CardHeader>
        <CardContent className="h-64 flex items-center justify-center">
          <div className="text-muted-foreground">加载中...</div>
        </CardContent>
      </Card>
    );
  }

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <div>
            <CardTitle>任务列表</CardTitle>
            <CardDescription>共 {data.length} 个任务</CardDescription>
          </div>
          <Button variant="outline" size="sm" onClick={onRefresh}>
            <RefreshCw className="h-4 w-4 mr-2" />
            刷新
          </Button>
        </div>
      </CardHeader>
      <CardContent>
        {data.length === 0 ? (
          <div className="text-center py-8 text-muted-foreground">
            暂无任务数据
          </div>
        ) : (
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead 
                  className="cursor-pointer hover:bg-muted/50" 
                  onClick={() => handleSort('name')}
                >
                  任务名称 {sortField === 'name' && (sortOrder === 'asc' ? '↑' : '↓')}
                </TableHead>
                <TableHead 
                  className="cursor-pointer hover:bg-muted/50" 
                  onClick={() => handleSort('groupName')}
                >
                  任务组 {sortField === 'groupName' && (sortOrder === 'asc' ? '↑' : '↓')}
                </TableHead>
                <TableHead 
                  className="cursor-pointer hover:bg-muted/50" 
                  onClick={() => handleSort('status')}
                >
                  状态 {sortField === 'status' && (sortOrder === 'asc' ? '↑' : '↓')}
                </TableHead>
                <TableHead 
                  className="cursor-pointer hover:bg-muted/50" 
                  onClick={() => handleSort('createTime')}
                >
                  创建时间 {sortField === 'createTime' && (sortOrder === 'asc' ? '↑' : '↓')}
                </TableHead>
                <TableHead>操作</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              {sortedData.map((task) => (
                <TableRow key={task.id}>
                  <TableCell className="font-medium">{task.name}</TableCell>
                  <TableCell>{task.groupName}</TableCell>
                  <TableCell>
                    <Badge variant={getStatusBadgeVariant(task.status)}>
                      {getStatusLabel(task.status)}
                    </Badge>
                  </TableCell>
                  <TableCell>{formatTimeAgo(task.createTime)}</TableCell>
                  <TableCell>
                    {task.status !== 'created' && task.id && (
                      <TaskLogsDialog taskId={task.id} taskName={task.name} />
                    )}
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        )}
      </CardContent>
    </Card>
  );
}
