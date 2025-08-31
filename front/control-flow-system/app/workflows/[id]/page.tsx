"use client"

import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { WorkflowYamlEditor } from "@/components/workflows/workflow-yaml-editor"
import { WorkflowWorkers } from "@/components/workflows/workflow-workers"
import { WorkflowLogs } from "@/components/workflows/workflow-logs"
import { SystemMetricsChart } from "@/components/dashboard/system-metrics-chart"
import { WorkflowCanvas } from "@/components/workflows/visual-editor/workflow-canvas"
import { NodePalette } from "@/components/workflows/visual-editor/node-palette"
import { NodeProperties } from "@/components/workflows/visual-editor/node-properties"
import { ArrowLeft, Play, Settings } from "lucide-react"
import Link from "next/link"

// Mock workflow data
const mockWorkflow = {
  id: "wf-001",
  name: "Data Processing Pipeline",
  status: "running",
  createdAt: "2024-01-15T10:30:00Z",
  updatedAt: "2024-01-20T14:22:00Z",
  workerCount: 3,
  description:
    "A comprehensive data processing pipeline that extracts, transforms, and loads data from multiple sources.",
}

const statusColors = {
  running: "bg-chart-4 text-white",
  stopped: "bg-muted text-muted-foreground",
  error: "bg-chart-3 text-white",
  pending: "bg-chart-2 text-white",
}

export default function WorkflowDetailPage({ params }: { params: { id: string } }) {
  return (
    <div className="space-y-6">
      <div className="flex items-center gap-4">
        <Link href="/workflows">
          <Button variant="ghost" size="icon">
            <ArrowLeft className="h-4 w-4" />
          </Button>
        </Link>
        <div className="flex-1">
          <div className="flex items-center gap-3">
            <h1 className="text-3xl font-bold text-foreground">{mockWorkflow.name}</h1>
            <Badge className={statusColors[mockWorkflow.status as keyof typeof statusColors]}>
              {mockWorkflow.status}
            </Badge>
          </div>
          <p className="text-muted-foreground mt-1">{mockWorkflow.description}</p>
        </div>
        <div className="flex gap-2">
          <Button variant="outline">
            <Settings className="h-4 w-4 mr-2" />
            Configure
          </Button>
          <Button>
            <Play className="h-4 w-4 mr-2" />
            Start
          </Button>
        </div>
      </div>

      <Tabs defaultValue="overview" className="w-full">
        <TabsList className="grid w-full grid-cols-4">
          <TabsTrigger value="overview">Overview</TabsTrigger>
          <TabsTrigger value="editor">Visual Editor</TabsTrigger>
          <TabsTrigger value="code">YAML & Logs</TabsTrigger>
          <TabsTrigger value="workers">Workers</TabsTrigger>
        </TabsList>

        <TabsContent value="overview" className="space-y-6">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <SystemMetricsChart title="Workflow CPU Usage" metric="cpu" color="hsl(var(--chart-1))" unit="%" />
            <SystemMetricsChart title="Workflow Memory Usage" metric="memory" color="hsl(var(--chart-2))" unit="%" />
          </div>
        </TabsContent>

        <TabsContent value="editor" className="space-y-6">
          <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
            <div className="lg:col-span-3">
              <WorkflowCanvas />
            </div>
            <div className="space-y-6">
              <NodePalette />
            </div>
          </div>
          <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
            <div className="lg:col-span-1">
              <NodeProperties selectedNode={null} />
            </div>
          </div>
        </TabsContent>

        <TabsContent value="code" className="space-y-6">
          <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
            <div className="space-y-6">
              <WorkflowYamlEditor />
              <WorkflowLogs />
            </div>
          </div>
        </TabsContent>

        <TabsContent value="workers" className="space-y-6">
          <WorkflowWorkers />
        </TabsContent>
      </Tabs>
    </div>
  )
}
