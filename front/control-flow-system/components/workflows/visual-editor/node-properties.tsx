"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Textarea } from "@/components/ui/textarea"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Badge } from "@/components/ui/badge"
import { Trash2, Copy } from "lucide-react"

interface NodePropertiesProps {
  selectedNode?: {
    id: string
    type: string
    name: string
    x: number
    y: number
  } | null
}

export function NodeProperties({ selectedNode }: NodePropertiesProps) {
  if (!selectedNode) {
    return (
      <Card className="h-[600px]">
        <CardHeader>
          <CardTitle>Properties</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="flex items-center justify-center h-full text-muted-foreground">
            Select a node to edit properties
          </div>
        </CardContent>
      </Card>
    )
  }

  return (
    <Card className="h-[600px]">
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle>Properties</CardTitle>
          <div className="flex gap-1">
            <Button variant="ghost" size="icon" className="h-8 w-8">
              <Copy className="h-4 w-4" />
            </Button>
            <Button variant="ghost" size="icon" className="h-8 w-8 text-destructive">
              <Trash2 className="h-4 w-4" />
            </Button>
          </div>
        </div>
      </CardHeader>
      <CardContent className="space-y-4 overflow-y-auto">
        <div className="flex items-center gap-2">
          <Badge variant="outline">{selectedNode.type}</Badge>
          <span className="text-sm text-muted-foreground">ID: {selectedNode.id}</span>
        </div>

        <div className="space-y-2">
          <Label htmlFor="node-name">Name</Label>
          <Input id="node-name" defaultValue={selectedNode.name} />
        </div>

        <div className="space-y-2">
          <Label htmlFor="node-description">Description</Label>
          <Textarea id="node-description" placeholder="Enter node description..." className="min-h-[80px]" />
        </div>

        {selectedNode.type === "input" && (
          <>
            <div className="space-y-2">
              <Label htmlFor="source-type">Source Type</Label>
              <Select defaultValue="database">
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="database">Database</SelectItem>
                  <SelectItem value="api">API</SelectItem>
                  <SelectItem value="file">File</SelectItem>
                  <SelectItem value="stream">Stream</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div className="space-y-2">
              <Label htmlFor="connection-string">Connection String</Label>
              <Input id="connection-string" placeholder="postgresql://user:pass@host:port/db" type="password" />
            </div>
          </>
        )}

        {selectedNode.type === "processor" && (
          <>
            <div className="space-y-2">
              <Label htmlFor="processor-type">Processor Type</Label>
              <Select defaultValue="transform">
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="transform">Transform</SelectItem>
                  <SelectItem value="filter">Filter</SelectItem>
                  <SelectItem value="aggregate">Aggregate</SelectItem>
                  <SelectItem value="validate">Validate</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div className="space-y-2">
              <Label htmlFor="processing-script">Processing Script</Label>
              <Textarea
                id="processing-script"
                placeholder="def process(data):\n    # Your processing logic here\n    return data"
                className="min-h-[120px] font-mono text-sm"
              />
            </div>
          </>
        )}

        {selectedNode.type === "output" && (
          <>
            <div className="space-y-2">
              <Label htmlFor="output-type">Output Type</Label>
              <Select defaultValue="file">
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="file">File</SelectItem>
                  <SelectItem value="database">Database</SelectItem>
                  <SelectItem value="api">API</SelectItem>
                  <SelectItem value="stream">Stream</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div className="space-y-2">
              <Label htmlFor="output-format">Format</Label>
              <Select defaultValue="json">
                <SelectTrigger>
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="json">JSON</SelectItem>
                  <SelectItem value="csv">CSV</SelectItem>
                  <SelectItem value="xml">XML</SelectItem>
                  <SelectItem value="parquet">Parquet</SelectItem>
                </SelectContent>
              </Select>
            </div>

            <div className="space-y-2">
              <Label htmlFor="output-path">Output Path</Label>
              <Input id="output-path" placeholder="/path/to/output" />
            </div>
          </>
        )}

        <div className="pt-4 border-t border-border">
          <h4 className="text-sm font-medium text-foreground mb-3">Runtime Settings</h4>
          <div className="space-y-3">
            <div className="grid grid-cols-2 gap-2">
              <div>
                <Label htmlFor="memory-limit" className="text-xs">
                  Memory (MB)
                </Label>
                <Input id="memory-limit" defaultValue="512" className="h-8" />
              </div>
              <div>
                <Label htmlFor="cpu-limit" className="text-xs">
                  CPU Cores
                </Label>
                <Input id="cpu-limit" defaultValue="0.5" className="h-8" />
              </div>
            </div>
            <div>
              <Label htmlFor="timeout" className="text-xs">
                Timeout (seconds)
              </Label>
              <Input id="timeout" defaultValue="300" className="h-8" />
            </div>
          </div>
        </div>

        <div className="pt-4">
          <Button className="w-full">Apply Changes</Button>
        </div>
      </CardContent>
    </Card>
  )
}
