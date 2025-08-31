"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Download, RefreshCw } from "lucide-react"

const mockLogs = [
  {
    timestamp: "2024-01-20T14:22:15Z",
    level: "INFO",
    message: "Workflow started successfully",
    source: "workflow-controller",
  },
  {
    timestamp: "2024-01-20T14:22:18Z",
    level: "INFO",
    message: "Task 'extract-data' started on worker data-processor-1",
    source: "task-scheduler",
  },
  {
    timestamp: "2024-01-20T14:22:45Z",
    level: "INFO",
    message: "Task 'extract-data' completed successfully",
    source: "data-processor-1",
  },
  {
    timestamp: "2024-01-20T14:22:46Z",
    level: "INFO",
    message: "Task 'transform-data' started on worker data-processor-2",
    source: "task-scheduler",
  },
  {
    timestamp: "2024-01-20T14:23:12Z",
    level: "WARN",
    message: "High memory usage detected on worker data-processor-2",
    source: "monitoring",
  },
  {
    timestamp: "2024-01-20T14:23:28Z",
    level: "INFO",
    message: "Task 'transform-data' completed successfully",
    source: "data-processor-2",
  },
]

const levelColors = {
  INFO: "bg-chart-1 text-white",
  WARN: "bg-chart-2 text-white",
  ERROR: "bg-chart-3 text-white",
  DEBUG: "bg-muted text-muted-foreground",
}

export function WorkflowLogs() {
  const formatTime = (timestamp: string) => {
    return new Date(timestamp).toLocaleTimeString("en-US", {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
    })
  }

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle>Runtime Logs</CardTitle>
          <div className="flex gap-2">
            <Button variant="outline" size="sm">
              <RefreshCw className="h-4 w-4 mr-2" />
              Refresh
            </Button>
            <Button variant="outline" size="sm">
              <Download className="h-4 w-4 mr-2" />
              Download
            </Button>
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <div className="space-y-2 max-h-96 overflow-y-auto">
          {mockLogs.map((log, index) => (
            <div key={index} className="flex items-start gap-3 p-3 bg-muted/50 rounded-md text-sm">
              <span className="text-muted-foreground font-mono text-xs">{formatTime(log.timestamp)}</span>
              <Badge className={levelColors[log.level as keyof typeof levelColors]} variant="secondary">
                {log.level}
              </Badge>
              <div className="flex-1">
                <p className="text-foreground">{log.message}</p>
                <p className="text-xs text-muted-foreground mt-1">Source: {log.source}</p>
              </div>
            </div>
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
