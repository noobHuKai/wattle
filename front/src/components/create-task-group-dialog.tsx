import { useState } from "react";
import { useMutation } from "@tanstack/react-query";
import { Dialog, DialogContent, DialogDescription, DialogFooter, DialogHeader, DialogTitle, DialogTrigger } from "@/components/ui/dialog";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { api } from "@/services/api";
import { toast } from "sonner";

interface CreateTaskGroupDialogProps {
  children: React.ReactNode;
  onSuccess?: () => void;
}

export function CreateTaskGroupDialog({ children, onSuccess }: CreateTaskGroupDialogProps) {
  const [open, setOpen] = useState(false);
  const [name, setName] = useState('');

  const createMutation = useMutation({
    mutationFn: (data: { name: string }) => api.createTaskGroup(data),
    onSuccess: () => {
      toast.success('任务组创建成功');
      setOpen(false);
      setName('');
      onSuccess?.();
    },
    onError: (error) => {
      toast.error('创建任务组失败: ' + error.message);
    },
  });

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!name.trim()) {
      toast.error('请输入任务组名称');
      return;
    }
    createMutation.mutate({ name: name.trim() });
  };

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        {children}
      </DialogTrigger>
      <DialogContent className="sm:max-w-[425px]">
        <DialogHeader>
          <DialogTitle>创建任务组</DialogTitle>
          <DialogDescription>
            创建一个新的任务组来管理相关任务。
          </DialogDescription>
        </DialogHeader>
        <form onSubmit={handleSubmit}>
          <div className="grid gap-4 py-4">
            <div className="grid grid-cols-4 items-center gap-4">
              <Label htmlFor="name" className="text-right">
                名称
              </Label>
              <Input
                id="name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                className="col-span-3"
                placeholder="输入任务组名称..."
                required
              />
            </div>
          </div>
          <DialogFooter>
            <Button type="button" variant="outline" onClick={() => setOpen(false)}>
              取消
            </Button>
            <Button type="submit" disabled={createMutation.isPending}>
              {createMutation.isPending ? '创建中...' : '创建'}
            </Button>
          </DialogFooter>
        </form>
      </DialogContent>
    </Dialog>
  );
}
