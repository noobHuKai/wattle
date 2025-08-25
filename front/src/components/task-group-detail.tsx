import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { ArrowLeft } from "lucide-react";
import { Link } from "@tanstack/react-router";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { formatTimeAgo } from "@/utils/time";
import type { TaskGroupDetail as TaskGroupDetailType } from "@/services/api";

interface TaskGroupDetailProps {
  data: TaskGroupDetailType;
}

const getStatusBadgeVariant = (status: string) => {
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

const getStatusLabel = (status: string) => {
  const statusLabels: Record<string, string> = {
    created: '已创建',
    running: '运行中',
    completed: '已完成',
    failed: '失败',
    cancelled: '已取消',
    active: '活跃',
    inactive: '非活跃',
  };
  return statusLabels[status] || status;
};

export function TaskGroupDetail({ data }: TaskGroupDetailProps) {
  return (
    <div className="space-y-6">
      {/* 返回按钮和标题 */}
      <div className="flex items-center gap-4">
        <Link to="/task_group">
          <Button variant="outline" size="sm">
            <ArrowLeft className="h-4 w-4 mr-2" />
            返回
          </Button>
        </Link>
        <div>
          <h1 className="text-3xl font-bold tracking-tight">{data.name}</h1>
          <p className="text-muted-foreground">任务组详情</p>
        </div>
      </div>

      {/* 任务组基本信息 */}
      <div className="grid gap-4 md:grid-cols-3">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">状态</CardTitle>
          </CardHeader>
          <CardContent>
            <Badge variant={getStatusBadgeVariant(data.status)}>
              {getStatusLabel(data.status)}
            </Badge>
          </CardContent>
        </Card>
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">任务数量</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{data.tasks.length}</div>
          </CardContent>
        </Card>
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">创建时间</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="text-lg">{formatTimeAgo(data.createTime)}</div>
          </CardContent>
        </Card>
      </div>

      {/* 任务列表 */}
      <Card>
        <CardHeader>
          <CardTitle>任务列表</CardTitle>
          <CardDescription>该任务组下的所有任务</CardDescription>
        </CardHeader>
        <CardContent>
          {data.tasks.length === 0 ? (
            <div className="text-center py-8 text-muted-foreground">
              该任务组下暂无任务
            </div>
          ) : (
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead>任务名称</TableHead>
                  <TableHead>状态</TableHead>
                  <TableHead>创建时间</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {data.tasks.map((task) => (
                  <TableRow key={task.id}>
                    <TableCell className="font-medium">{task.name}</TableCell>
                    <TableCell>
                      <Badge variant={getStatusBadgeVariant(task.status)}>
                        {getStatusLabel(task.status)}
                      </Badge>
                    </TableCell>
                    <TableCell>{formatTimeAgo(task.createTime)}</TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          )}
        </CardContent>
      </Card>

      {/* Topic列表 */}
      <Card>
        <CardHeader>
          <CardTitle>Topics</CardTitle>
          <CardDescription>该任务组相关的主题列表</CardDescription>
        </CardHeader>
        <CardContent>
          {data.topics.length === 0 ? (
            <div className="text-center py-8 text-muted-foreground">
              暂无相关主题
            </div>
          ) : (
            <div className="flex flex-wrap gap-2">
              {data.topics.map((topic, index) => (
                <Badge key={index} variant="outline">
                  {topic}
                </Badge>
              ))}
            </div>
          )}
        </CardContent>
      </Card>
    </div>
  );
}
