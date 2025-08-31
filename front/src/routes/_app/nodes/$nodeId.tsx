import { createFileRoute, Link } from '@tanstack/react-router'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { ArrowLeft, Code, Settings, Play } from 'lucide-react'

// Mock node data
const mockNode = {
  id: "node-001",
  name: "Data Extractor",
  type: "input",
  version: "1.2.0",
  status: "active",
  createdAt: "2024-01-15T10:30:00Z",
  updatedAt: "2024-01-20T14:22:00Z",
  description: "Extracts data from various external sources including APIs, databases, and file systems.",
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

function NodeDetail() {
  const { nodeId } = Route.useParams()
  
  return (
    <div className="space-y-6">
      <div className="flex items-center gap-4">
        <Link to="/nodes">
          <Button variant="ghost" size="icon">
            <ArrowLeft className="h-4 w-4" />
          </Button>
        </Link>
        <div className="flex-1">
          <div className="flex items-center gap-3">
            <h1 className="text-3xl font-bold text-foreground">{mockNode.name}</h1>
            <Badge className={typeColors[mockNode.type as keyof typeof typeColors]}>
              {mockNode.type}
            </Badge>
            <Badge className={statusColors[mockNode.status as keyof typeof statusColors]}>
              {mockNode.status}
            </Badge>
          </div>
          <p className="text-muted-foreground mt-1">{mockNode.description}</p>
        </div>
        <div className="flex items-center gap-2">
          <Button>
            <Play className="h-4 w-4 mr-2" />
            Test
          </Button>
          <Button variant="outline">
            <Settings className="h-4 w-4 mr-2" />
            Configure
          </Button>
        </div>
      </div>

      <Tabs defaultValue="info" className="space-y-4">
        <TabsList>
          <TabsTrigger value="info">Info</TabsTrigger>
          <TabsTrigger value="source">Source Code</TabsTrigger>
          <TabsTrigger value="schema">Schema</TabsTrigger>
          <TabsTrigger value="directory">Directory</TabsTrigger>
        </TabsList>

        <TabsContent value="info" className="space-y-4">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
            <div className="bg-card p-4 rounded-lg border border-border">
              <div className="text-sm text-muted-foreground">Version</div>
              <div className="text-xl font-bold text-foreground font-mono">{mockNode.version}</div>
            </div>
            <div className="bg-card p-4 rounded-lg border border-border">
              <div className="text-sm text-muted-foreground">Type</div>
              <div className="text-xl font-bold text-foreground">{mockNode.type}</div>
            </div>
            <div className="bg-card p-4 rounded-lg border border-border">
              <div className="text-sm text-muted-foreground">Status</div>
              <div className="text-xl font-bold text-foreground">{mockNode.status}</div>
            </div>
            <div className="bg-card p-4 rounded-lg border border-border">
              <div className="text-sm text-muted-foreground">Created</div>
              <div className="text-xl font-bold text-foreground">
                {new Date(mockNode.createdAt).toLocaleDateString()}
              </div>
            </div>
          </div>
        </TabsContent>

        <TabsContent value="source">
          <div className="bg-card p-6 rounded-lg border border-border">
            <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
              <Code className="h-5 w-5" />
              Source Code
            </h3>
            <p className="text-muted-foreground">Source code editor will be implemented here.</p>
          </div>
        </TabsContent>

        <TabsContent value="schema">
          <div className="bg-card p-6 rounded-lg border border-border">
            <h3 className="text-lg font-semibold mb-4">Schema Definition</h3>
            <p className="text-muted-foreground">Schema editor will be implemented here.</p>
          </div>
        </TabsContent>

        <TabsContent value="directory">
          <div className="bg-card p-6 rounded-lg border border-border">
            <h3 className="text-lg font-semibold mb-4">Directory Structure</h3>
            <p className="text-muted-foreground">Directory tree will be implemented here.</p>
          </div>
        </TabsContent>
      </Tabs>
    </div>
  )
}

export const Route = createFileRoute('/_app/nodes/$nodeId')({
  component: NodeDetail,
})
