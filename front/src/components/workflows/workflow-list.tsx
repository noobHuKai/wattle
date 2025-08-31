"use client"

import { useState } from "react"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Badge } from "@/components/ui/badge"
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table"
import { DropdownMenu, DropdownMenuContent, DropdownMenuItem, DropdownMenuTrigger } from "@/components/ui/dropdown-menu"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Search, Plus, MoreHorizontal, Play, Pause, Settings, Trash2 } from "lucide-react"
import { Link } from "@tanstack/react-router"

// Mock data for workflows
const mockWorkflows = [
  {
    id: "wf-001",
    name: "Data Processing Pipeline",
    status: "running",
    createdAt: "2024-01-15T10:30:00Z",
    updatedAt: "2024-01-20T14:22:00Z",
    workerCount: 3,
  },
  {
    id: "wf-002",
    name: "API Gateway Workflow",
    status: "stopped",
    createdAt: "2024-01-10T09:15:00Z",
    updatedAt: "2024-01-19T16:45:00Z",
    workerCount: 2,
  },
  {
    id: "wf-003",
    name: "Batch Processing Job",
    status: "error",
    createdAt: "2024-01-12T11:20:00Z",
    updatedAt: "2024-01-20T08:30:00Z",
    workerCount: 1,
  },
  {
    id: "wf-004",
    name: "Real-time Analytics",
    status: "running",
    createdAt: "2024-01-18T13:45:00Z",
    updatedAt: "2024-01-20T12:15:00Z",
    workerCount: 4,
  },
]

const statusColors = {
  running: "bg-chart-4 text-white",
  stopped: "bg-muted text-muted-foreground",
  error: "bg-chart-3 text-white",
  pending: "bg-chart-2 text-white",
}

export function WorkflowList() {
  const [searchTerm, setSearchTerm] = useState("")
  const [statusFilter, setStatusFilter] = useState("all")

  const filteredWorkflows = mockWorkflows.filter((workflow) => {
    const matchesSearch = workflow.name.toLowerCase().includes(searchTerm.toLowerCase())
    const matchesStatus = statusFilter === "all" || workflow.status === statusFilter
    return matchesSearch && matchesStatus
  })

  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString("en-US", {
      year: "numeric",
      month: "short",
      day: "numeric",
      hour: "2-digit",
      minute: "2-digit",
    })
  }

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle>Workflows</CardTitle>
          <Button className="gap-2">
            <Plus className="h-4 w-4" />
            New Workflow
          </Button>
        </div>
        <div className="flex items-center gap-4">
          <div className="relative flex-1 max-w-sm">
            <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
            <Input
              placeholder="Search workflows..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="pl-10"
            />
          </div>
          <DropdownMenu>
            <DropdownMenuTrigger asChild>
              <Button variant="outline">Status: {statusFilter === "all" ? "All" : statusFilter}</Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent>
              <DropdownMenuItem onClick={() => setStatusFilter("all")}>All</DropdownMenuItem>
              <DropdownMenuItem onClick={() => setStatusFilter("running")}>Running</DropdownMenuItem>
              <DropdownMenuItem onClick={() => setStatusFilter("stopped")}>Stopped</DropdownMenuItem>
              <DropdownMenuItem onClick={() => setStatusFilter("error")}>Error</DropdownMenuItem>
            </DropdownMenuContent>
          </DropdownMenu>
        </div>
      </CardHeader>
      <CardContent>
        <Table>
          <TableHeader>
            <TableRow>
              <TableHead>Name</TableHead>
              <TableHead>Status</TableHead>
              <TableHead>Workers</TableHead>
              <TableHead>Created</TableHead>
              <TableHead>Updated</TableHead>
              <TableHead className="w-[50px]"></TableHead>
            </TableRow>
          </TableHeader>
          <TableBody>
            {filteredWorkflows.map((workflow) => (
              <TableRow key={workflow.id}>
                <TableCell>
                  <Link
                    to="/workflows/$workflowId"
                    params={{ workflowId: workflow.id }}
                    className="font-medium text-foreground hover:text-accent transition-colors"
                  >
                    {workflow.name}
                  </Link>
                </TableCell>
                <TableCell>
                  <Badge className={statusColors[workflow.status as keyof typeof statusColors]}>
                    {workflow.status}
                  </Badge>
                </TableCell>
                <TableCell>{workflow.workerCount}</TableCell>
                <TableCell className="text-muted-foreground">{formatDate(workflow.createdAt)}</TableCell>
                <TableCell className="text-muted-foreground">{formatDate(workflow.updatedAt)}</TableCell>
                <TableCell>
                  <DropdownMenu>
                    <DropdownMenuTrigger asChild>
                      <Button variant="ghost" size="icon" className="h-8 w-8">
                        <MoreHorizontal className="h-4 w-4" />
                      </Button>
                    </DropdownMenuTrigger>
                    <DropdownMenuContent align="end">
                      <DropdownMenuItem>
                        <Play className="mr-2 h-4 w-4" />
                        Start
                      </DropdownMenuItem>
                      <DropdownMenuItem>
                        <Pause className="mr-2 h-4 w-4" />
                        Stop
                      </DropdownMenuItem>
                      <DropdownMenuItem>
                        <Settings className="mr-2 h-4 w-4" />
                        Configure
                      </DropdownMenuItem>
                      <DropdownMenuItem className="text-destructive">
                        <Trash2 className="mr-2 h-4 w-4" />
                        Delete
                      </DropdownMenuItem>
                    </DropdownMenuContent>
                  </DropdownMenu>
                </TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      </CardContent>
    </Card>
  )
}
