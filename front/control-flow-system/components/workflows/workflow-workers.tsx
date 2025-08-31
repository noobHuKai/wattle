"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Cpu, HardDrive, MoreHorizontal } from "lucide-react"
import { DropdownMenu, DropdownMenuContent, DropdownMenuItem, DropdownMenuTrigger } from "@/components/ui/dropdown-menu"

const mockWorkers = [
  {
    id: "worker-001",
    name: "data-processor-1",
    status: "running",
    cpu: 45.2,
    memory: 62.8,
    uptime: "2h 15m",
  },
  {
    id: "worker-002",
    name: "data-processor-2",
    status: "running",
    cpu: 38.7,
    memory: 58.3,
    uptime: "2h 15m",
  },
  {
    id: "worker-003",
    name: "data-processor-3",
    status: "idle",
    cpu: 12.1,
    memory: 34.5,
    uptime: "2h 15m",
  },
]

const statusColors = {
  running: "bg-chart-4 text-white",
  idle: "bg-chart-2 text-white",
  error: "bg-chart-3 text-white",
}

export function WorkflowWorkers() {
  return (
    <Card>
      <CardHeader>
        <CardTitle>Workers</CardTitle>
      </CardHeader>
      <CardContent>
        <div className="space-y-4">
          {mockWorkers.map((worker) => (
            <div key={worker.id} className="flex items-center justify-between p-4 border border-border rounded-lg">
              <div className="flex items-center gap-4">
                <div>
                  <h4 className="font-medium text-foreground">{worker.name}</h4>
                  <p className="text-sm text-muted-foreground">Uptime: {worker.uptime}</p>
                </div>
                <Badge className={statusColors[worker.status as keyof typeof statusColors]}>{worker.status}</Badge>
              </div>

              <div className="flex items-center gap-6">
                <div className="flex items-center gap-2 text-sm">
                  <Cpu className="h-4 w-4 text-chart-1" />
                  <span>{worker.cpu}%</span>
                </div>
                <div className="flex items-center gap-2 text-sm">
                  <HardDrive className="h-4 w-4 text-chart-2" />
                  <span>{worker.memory}%</span>
                </div>
                <DropdownMenu>
                  <DropdownMenuTrigger asChild>
                    <Button variant="ghost" size="icon" className="h-8 w-8">
                      <MoreHorizontal className="h-4 w-4" />
                    </Button>
                  </DropdownMenuTrigger>
                  <DropdownMenuContent align="end">
                    <DropdownMenuItem>View Logs</DropdownMenuItem>
                    <DropdownMenuItem>Restart</DropdownMenuItem>
                    <DropdownMenuItem className="text-destructive">Terminate</DropdownMenuItem>
                  </DropdownMenuContent>
                </DropdownMenu>
              </div>
            </div>
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
