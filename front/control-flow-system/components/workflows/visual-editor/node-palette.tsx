"use client"

import type React from "react"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Database, Cpu, FileOutput, Globe, Filter, Zap } from "lucide-react"

const nodeTypes = [
  {
    type: "input",
    name: "Data Source",
    icon: Database,
    description: "Extract data from databases, APIs, or files",
    color: "bg-chart-1",
  },
  {
    type: "input",
    name: "HTTP Input",
    icon: Globe,
    description: "Receive data via HTTP requests",
    color: "bg-chart-1",
  },
  {
    type: "processor",
    name: "Transform",
    icon: Cpu,
    description: "Process and transform data",
    color: "bg-chart-2",
  },
  {
    type: "processor",
    name: "Filter",
    icon: Filter,
    description: "Filter data based on conditions",
    color: "bg-chart-2",
  },
  {
    type: "processor",
    name: "Aggregator",
    icon: Zap,
    description: "Aggregate and summarize data",
    color: "bg-chart-2",
  },
  {
    type: "output",
    name: "File Output",
    icon: FileOutput,
    description: "Write data to files or storage",
    color: "bg-chart-4",
  },
]

export function NodePalette() {
  const handleDragStart = (e: React.DragEvent, nodeType: any) => {
    e.dataTransfer.setData("application/json", JSON.stringify(nodeType))
  }

  return (
    <Card className="h-[600px]">
      <CardHeader>
        <CardTitle>Node Palette</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3 overflow-y-auto">
        {nodeTypes.map((nodeType, index) => (
          <div
            key={index}
            className="p-3 border border-border rounded-lg cursor-grab hover:bg-muted/50 transition-colors"
            draggable
            onDragStart={(e) => handleDragStart(e, nodeType)}
          >
            <div className="flex items-start gap-3">
              <div className={`p-2 rounded-md ${nodeType.color}`}>
                <nodeType.icon className="h-4 w-4 text-white" />
              </div>
              <div className="flex-1 min-w-0">
                <div className="flex items-center gap-2">
                  <h4 className="text-sm font-medium text-foreground">{nodeType.name}</h4>
                  <Badge variant="outline" className="text-xs">
                    {nodeType.type}
                  </Badge>
                </div>
                <p className="text-xs text-muted-foreground mt-1">{nodeType.description}</p>
              </div>
            </div>
          </div>
        ))}
      </CardContent>
    </Card>
  )
}
