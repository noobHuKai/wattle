"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"

interface NodeInfoTabProps {
  node: {
    id: string
    name: string
    type: string
    version: string
    status: string
    createdAt: string
    updatedAt: string
    description: string
  }
}

const typeColors = {
  input: "bg-chart-1 text-white",
  processor: "bg-chart-2 text-white",
  output: "bg-chart-4 text-white",
}

const statusColors = {
  active: "bg-chart-4 text-white",
  deprecated: "bg-chart-3 text-white",
  draft: "bg-muted text-muted-foreground",
}

export function NodeInfoTab({ node }: NodeInfoTabProps) {
  return (
    <div className="space-y-6">
      <Card>
        <CardHeader>
          <CardTitle>Basic Information</CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="grid grid-cols-2 gap-4">
            <div>
              <label className="text-sm font-medium text-muted-foreground">Name</label>
              <p className="text-sm text-foreground mt-1">{node.name}</p>
            </div>
            <div>
              <label className="text-sm font-medium text-muted-foreground">ID</label>
              <p className="text-sm text-foreground mt-1 font-mono">{node.id}</p>
            </div>
            <div>
              <label className="text-sm font-medium text-muted-foreground">Type</label>
              <div className="mt-1">
                <Badge className={typeColors[node.type as keyof typeof typeColors]}>{node.type}</Badge>
              </div>
            </div>
            <div>
              <label className="text-sm font-medium text-muted-foreground">Status</label>
              <div className="mt-1">
                <Badge className={statusColors[node.status as keyof typeof statusColors]}>{node.status}</Badge>
              </div>
            </div>
            <div>
              <label className="text-sm font-medium text-muted-foreground">Version</label>
              <p className="text-sm text-foreground mt-1 font-mono">v{node.version}</p>
            </div>
            <div>
              <label className="text-sm font-medium text-muted-foreground">Runtime</label>
              <p className="text-sm text-foreground mt-1">Python 3.11</p>
            </div>
          </div>
          <div>
            <label className="text-sm font-medium text-muted-foreground">Description</label>
            <p className="text-sm text-foreground mt-1">{node.description}</p>
          </div>
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>Timestamps</CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="grid grid-cols-2 gap-4">
            <div>
              <label className="text-sm font-medium text-muted-foreground">Created At</label>
              <p className="text-sm text-foreground mt-1">{new Date(node.createdAt).toLocaleString()}</p>
            </div>
            <div>
              <label className="text-sm font-medium text-muted-foreground">Last Modified</label>
              <p className="text-sm text-foreground mt-1">{new Date(node.updatedAt).toLocaleString()}</p>
            </div>
          </div>
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>Resource Limits</CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="grid grid-cols-2 gap-4">
            <div>
              <label className="text-sm font-medium text-muted-foreground">Memory Limit</label>
              <p className="text-sm text-foreground mt-1">512 MB</p>
            </div>
            <div>
              <label className="text-sm font-medium text-muted-foreground">CPU Limit</label>
              <p className="text-sm text-foreground mt-1">0.5 cores</p>
            </div>
            <div>
              <label className="text-sm font-medium text-muted-foreground">Timeout</label>
              <p className="text-sm text-foreground mt-1">300 seconds</p>
            </div>
            <div>
              <label className="text-sm font-medium text-muted-foreground">Max Instances</label>
              <p className="text-sm text-foreground mt-1">10</p>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
