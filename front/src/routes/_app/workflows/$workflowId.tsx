import { createFileRoute, Link } from '@tanstack/react-router'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { ArrowLeft, Play, Settings } from 'lucide-react'

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

function WorkflowDetail() {
  const { workflowId } = Route.useParams()
  
  return (
    <div className="space-y-6">
      <div className="flex items-center gap-4">
        <Link to="/workflows">
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
        <div className="flex items-center gap-2">
          <Button>
            <Play className="h-4 w-4 mr-2" />
            Run
          </Button>
          <Button variant="outline">
            <Settings className="h-4 w-4 mr-2" />
            Configure
          </Button>
        </div>
      </div>

      <Tabs defaultValue="overview" className="space-y-4">
        <TabsList>
          <TabsTrigger value="overview">Overview</TabsTrigger>
          <TabsTrigger value="editor">Visual Editor</TabsTrigger>
          <TabsTrigger value="yaml">YAML Config</TabsTrigger>
          <TabsTrigger value="workers">Workers</TabsTrigger>
          <TabsTrigger value="logs">Logs</TabsTrigger>
        </TabsList>

        <TabsContent value="overview" className="space-y-4">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
            <div className="bg-card p-4 rounded-lg border border-border">
              <div className="text-sm text-muted-foreground">Status</div>
              <div className="text-xl font-bold text-foreground">{mockWorkflow.status}</div>
            </div>
            <div className="bg-card p-4 rounded-lg border border-border">
              <div className="text-sm text-muted-foreground">Workers</div>
              <div className="text-xl font-bold text-foreground">{mockWorkflow.workerCount}</div>
            </div>
            <div className="bg-card p-4 rounded-lg border border-border">
              <div className="text-sm text-muted-foreground">Created</div>
              <div className="text-xl font-bold text-foreground">
                {new Date(mockWorkflow.createdAt).toLocaleDateString()}
              </div>
            </div>
            <div className="bg-card p-4 rounded-lg border border-border">
              <div className="text-sm text-muted-foreground">Last Updated</div>
              <div className="text-xl font-bold text-foreground">
                {new Date(mockWorkflow.updatedAt).toLocaleDateString()}
              </div>
            </div>
          </div>
        </TabsContent>

        <TabsContent value="editor">
          <div className="bg-card p-6 rounded-lg border border-border">
            <h3 className="text-lg font-semibold mb-4">Visual Workflow Editor</h3>
            <p className="text-muted-foreground">Visual workflow editor will be implemented here.</p>
          </div>
        </TabsContent>

        <TabsContent value="yaml">
          <div className="bg-card p-6 rounded-lg border border-border">
            <h3 className="text-lg font-semibold mb-4">YAML Configuration</h3>
            <p className="text-muted-foreground">YAML editor will be implemented here.</p>
          </div>
        </TabsContent>

        <TabsContent value="workers">
          <div className="bg-card p-6 rounded-lg border border-border">
            <h3 className="text-lg font-semibold mb-4">Workers</h3>
            <p className="text-muted-foreground">Worker management interface will be implemented here.</p>
          </div>
        </TabsContent>

        <TabsContent value="logs">
          <div className="bg-card p-6 rounded-lg border border-border">
            <h3 className="text-lg font-semibold mb-4">Logs</h3>
            <p className="text-muted-foreground">Log viewer will be implemented here.</p>
          </div>
        </TabsContent>
      </Tabs>
    </div>
  )
}

export const Route = createFileRoute('/_app/workflows/$workflowId')({
  component: WorkflowDetail,
})
