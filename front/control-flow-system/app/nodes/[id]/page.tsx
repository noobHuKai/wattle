"use client"

import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { NodeInfoTab } from "@/components/nodes/node-info-tab"
import { NodeSchemaEditor } from "@/components/nodes/node-schema-editor"
import { NodeSourceCodeTab } from "@/components/nodes/node-source-code-tab"
import { ArrowLeft, Settings, Combine as Compile, Download } from "lucide-react"
import Link from "next/link"

// Mock node data
const mockNode = {
  id: "node-001",
  name: "Data Extractor",
  type: "input",
  version: "1.2.0",
  status: "active",
  createdAt: "2024-01-15T10:30:00Z",
  updatedAt: "2024-01-20T14:22:00Z",
  description:
    "Extracts data from various sources including databases, APIs, and files with configurable output formats.",
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

export default function NodeDetailPage({ params }: { params: { id: string } }) {
  return (
    <div className="space-y-6">
      <div className="flex items-center gap-4">
        <Link href="/nodes">
          <Button variant="ghost" size="icon">
            <ArrowLeft className="h-4 w-4" />
          </Button>
        </Link>
        <div className="flex-1">
          <div className="flex items-center gap-3">
            <h1 className="text-3xl font-bold text-foreground">{mockNode.name}</h1>
            <Badge className={typeColors[mockNode.type as keyof typeof typeColors]}>{mockNode.type}</Badge>
            <Badge className={statusColors[mockNode.status as keyof typeof statusColors]}>{mockNode.status}</Badge>
            <span className="text-sm text-muted-foreground font-mono">v{mockNode.version}</span>
          </div>
          <p className="text-muted-foreground mt-1">{mockNode.description}</p>
        </div>
        <div className="flex gap-2">
          <Button variant="outline">
            <Download className="h-4 w-4 mr-2" />
            Export
          </Button>
          <Button variant="outline">
            <Compile className="h-4 w-4 mr-2" />
            Compile
          </Button>
          <Button>
            <Settings className="h-4 w-4 mr-2" />
            Configure
          </Button>
        </div>
      </div>

      <Tabs defaultValue="info" className="w-full">
        <TabsList className="grid w-full grid-cols-3">
          <TabsTrigger value="info">Information</TabsTrigger>
          <TabsTrigger value="schema">JSON Schema</TabsTrigger>
          <TabsTrigger value="source">Source Code</TabsTrigger>
        </TabsList>

        <TabsContent value="info" className="mt-6">
          <NodeInfoTab node={mockNode} />
        </TabsContent>

        <TabsContent value="schema" className="mt-6">
          <NodeSchemaEditor />
        </TabsContent>

        <TabsContent value="source" className="mt-6">
          <NodeSourceCodeTab />
        </TabsContent>
      </Tabs>
    </div>
  )
}
