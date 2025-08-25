import { useState } from "react";
import { Link } from "@tanstack/react-router";
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
import { RefreshCw, Eye } from "lucide-react";
import { formatTimeAgo } from "@/utils/time";
import type { TaskGroup } from "@/services/api";

interface TaskGroupTableProps {
  data: TaskGroup[];
  isLoading: boolean;
  onRefresh: () => void;
}

const getStatusBadgeVariant = (status: TaskGroup['status']) => {
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
    case 'active':
      return 'default';
    case 'inactive':
      return 'secondary';
    default:
      return 'secondary';
  }
};

const getStatusLabel = (status: TaskGroup['status']) => {
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
    case 'active':
      return '活跃';
    case 'inactive':
      return '非活跃';
    default:
      return status;
  }
};

export function TaskGroupTable({ data, isLoading, onRefresh }: TaskGroupTableProps) {
  const [sortField, setSortField] = useState<keyof TaskGroup>('createTime');
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

  const handleSort = (field: keyof TaskGroup) => {
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
          <CardTitle>任务组列表</CardTitle>
          <CardDescription>所有任务组的状态和详情</CardDescription>
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
            <CardTitle>任务组列表</CardTitle>
            <CardDescription>共 {data.length} 个任务组</CardDescription>
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
            暂无任务组数据
          </div>
        ) : (
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead 
                  className="cursor-pointer hover:bg-muted/50" 
                  onClick={() => handleSort('name')}
                >
                  任务组名称 {sortField === 'name' && (sortOrder === 'asc' ? '↑' : '↓')}
                </TableHead>
                <TableHead 
                  className="cursor-pointer hover:bg-muted/50" 
                  onClick={() => handleSort('status')}
                >
                  状态 {sortField === 'status' && (sortOrder === 'asc' ? '↑' : '↓')}
                </TableHead>
                <TableHead>任务数量</TableHead>
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
              {sortedData.map((group) => (
                <TableRow key={group.id}>
                  <TableCell className="font-medium">{group.name}</TableCell>
                  <TableCell>
                    <Badge variant={getStatusBadgeVariant(group.status)}>
                      {getStatusLabel(group.status)}
                    </Badge>
                  </TableCell>
                  <TableCell>{group.taskCount || 0}</TableCell>
                  <TableCell>{formatTimeAgo(group.createTime)}</TableCell>
                  <TableCell>
                    <Link to="/task_group/$groupId" params={{ groupId: group.id || group.name }}>
                      <Button variant="outline" size="sm">
                        <Eye className="h-4 w-4 mr-2" />
                        详情
                      </Button>
                    </Link>
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
