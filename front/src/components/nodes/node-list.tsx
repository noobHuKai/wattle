"use client"

import { useState } from "react"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Badge } from "@/components/ui/badge"
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table"
import { DropdownMenu, DropdownMenuContent, DropdownMenuItem, DropdownMenuTrigger } from "@/components/ui/dropdown-menu"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Search, Plus, MoreHorizontal, Code, Settings, Trash2, Combine as Compile } from "lucide-react"
import { Link } from "@tanstack/react-router"

// Mock data for nodes
const mockNodes = [
  {
    id: "node-001",
    name: "Data Extractor",
    type: "input",
    version: "1.2.0",
    createdAt: "2024-01-15T10:30:00Z",
    updatedAt: "2024-01-20T14:22:00Z",
    status: "active",
  },
  {
    id: "node-002",
    name: "Data Transformer",
    type: "processor",
    version: "2.1.0",
    createdAt: "2024-01-10T09:15:00Z",
    updatedAt: "2024-01-19T16:45:00Z",
    status: "active",
  },
  {
    id: "node-003",
    name: "Data Validator",
    type: "processor",
    version: "1.0.0",
    createdAt: "2024-01-12T11:20:00Z",
    updatedAt: "2024-01-18T08:30:00Z",
    status: "deprecated",
  },
  {
    id: "node-004",
    name: "Output Writer",
    type: "output",
    version: "1.5.0",
    createdAt: "2024-01-18T13:45:00Z",
    updatedAt: "2024-01-20T12:15:00Z",
    status: "active",
  },
]

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

export function NodeList() {
  const [searchTerm, setSearchTerm] = useState("")
  const [typeFilter, setTypeFilter] = useState("all")

  const filteredNodes = mockNodes.filter((node) => {
    const matchesSearch = node.name.toLowerCase().includes(searchTerm.toLowerCase())
    const matchesType = typeFilter === "all" || node.type === typeFilter
    return matchesSearch && matchesType
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
          <CardTitle>Nodes</CardTitle>
          <Button className="gap-2">
            <Plus className="h-4 w-4" />
            New Node
          </Button>
        </div>
        <div className="flex items-center gap-4">
          <div className="relative flex-1 max-w-sm">
            <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
            <Input
              placeholder="Search nodes..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="pl-10"
            />
          </div>
          <DropdownMenu>
            <DropdownMenuTrigger asChild>
              <Button variant="outline">Type: {typeFilter === "all" ? "All" : typeFilter}</Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent>
              <DropdownMenuItem onClick={() => setTypeFilter("all")}>All</DropdownMenuItem>
              <DropdownMenuItem onClick={() => setTypeFilter("input")}>Input</DropdownMenuItem>
              <DropdownMenuItem onClick={() => setTypeFilter("processor")}>Processor</DropdownMenuItem>
              <DropdownMenuItem onClick={() => setTypeFilter("output")}>Output</DropdownMenuItem>
            </DropdownMenuContent>
          </DropdownMenu>
        </div>
      </CardHeader>
      <CardContent>
        <Table>
          <TableHeader>
            <TableRow>
              <TableHead>Name</TableHead>
              <TableHead>Type</TableHead>
              <TableHead>Version</TableHead>
              <TableHead>Status</TableHead>
              <TableHead>Created</TableHead>
              <TableHead>Updated</TableHead>
              <TableHead className="w-[50px]"></TableHead>
            </TableRow>
          </TableHeader>
          <TableBody>
            {filteredNodes.map((node) => (
              <TableRow key={node.id}>
                <TableCell>
                  <Link
                    to="/nodes/$nodeId"
                    params={{ nodeId: node.id }}
                    className="font-medium text-foreground hover:text-accent transition-colors"
                  >
                    {node.name}
                  </Link>
                </TableCell>
                <TableCell>
                  <Badge className={typeColors[node.type as keyof typeof typeColors]}>{node.type}</Badge>
                </TableCell>
                <TableCell className="font-mono text-sm">{node.version}</TableCell>
                <TableCell>
                  <Badge className={statusColors[node.status as keyof typeof statusColors]}>{node.status}</Badge>
                </TableCell>
                <TableCell className="text-muted-foreground">{formatDate(node.createdAt)}</TableCell>
                <TableCell className="text-muted-foreground">{formatDate(node.updatedAt)}</TableCell>
                <TableCell>
                  <DropdownMenu>
                    <DropdownMenuTrigger asChild>
                      <Button variant="ghost" size="icon" className="h-8 w-8">
                        <MoreHorizontal className="h-4 w-4" />
                      </Button>
                    </DropdownMenuTrigger>
                    <DropdownMenuContent align="end">
                      <DropdownMenuItem>
                        <Code className="mr-2 h-4 w-4" />
                        View Code
                      </DropdownMenuItem>
                      <DropdownMenuItem>
                        <Compile className="mr-2 h-4 w-4" />
                        Compile
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
