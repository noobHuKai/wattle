import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";

interface TaskGroupFiltersProps {
  filters: {
    name?: string;
    status?: string;
  };
  onFiltersChange: (filters: TaskGroupFiltersProps['filters']) => void;
}

const STATUS_OPTIONS = [
  { value: 'all', label: '全部状态' },
  { value: 'created', label: '已创建' },
  { value: 'running', label: '运行中' },
  { value: 'completed', label: '已完成' },
  { value: 'failed', label: '失败' },
  { value: 'cancelled', label: '已取消' },
  { value: 'active', label: '活跃' },
  { value: 'inactive', label: '非活跃' },
];

export function TaskGroupFilters({ filters, onFiltersChange }: TaskGroupFiltersProps) {
  const updateFilter = (key: keyof TaskGroupFiltersProps['filters'], value: string) => {
    onFiltersChange({
      ...filters,
      [key]: value || undefined,
    });
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>筛选条件</CardTitle>
      </CardHeader>
      <CardContent>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div className="space-y-2">
            <Label htmlFor="name-filter">任务组名称</Label>
            <Input
              id="name-filter"
              placeholder="搜索任务组名称..."
              value={filters.name || ''}
              onChange={(e) => updateFilter('name', e.target.value)}
            />
          </div>
          <div className="space-y-2">
            <Label htmlFor="status-filter">状态</Label>
            <Select
              value={filters.status || ''}
              onValueChange={(value) => updateFilter('status', value)}
            >
              <SelectTrigger>
                <SelectValue placeholder="选择状态" />
              </SelectTrigger>
              <SelectContent>
                {STATUS_OPTIONS.map((option) => (
                  <SelectItem key={option.value} value={option.value}>
                    {option.label}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
