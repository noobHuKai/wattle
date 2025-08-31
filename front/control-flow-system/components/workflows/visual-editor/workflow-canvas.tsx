"use client"

import type React from "react"

import { useState, useRef, useCallback } from "react"
import { Card } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { ZoomIn, ZoomOut, Maximize, Save } from "lucide-react"

interface Node {
  id: string
  type: string
  name: string
  x: number
  y: number
  width: number
  height: number
}

interface Connection {
  id: string
  from: string
  to: string
  fromPort: string
  toPort: string
}

const initialNodes: Node[] = [
  { id: "node-1", type: "input", name: "Data Source", x: 100, y: 100, width: 120, height: 60 },
  { id: "node-2", type: "processor", name: "Transform", x: 300, y: 100, width: 120, height: 60 },
  { id: "node-3", type: "output", name: "Output", x: 500, y: 100, width: 120, height: 60 },
]

const initialConnections: Connection[] = [
  { id: "conn-1", from: "node-1", to: "node-2", fromPort: "output", toPort: "input" },
  { id: "conn-2", from: "node-2", to: "node-3", fromPort: "output", toPort: "input" },
]

const nodeColors = {
  input: "bg-chart-1 border-chart-1",
  processor: "bg-chart-2 border-chart-2",
  output: "bg-chart-4 border-chart-4",
}

export function WorkflowCanvas() {
  const [nodes, setNodes] = useState<Node[]>(initialNodes)
  const [connections, setConnections] = useState<Connection[]>(initialConnections)
  const [selectedNode, setSelectedNode] = useState<string | null>(null)
  const [draggedNode, setDraggedNode] = useState<string | null>(null)
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 })
  const [zoom, setZoom] = useState(1)
  const canvasRef = useRef<HTMLDivElement>(null)

  const handleNodeMouseDown = useCallback(
    (e: React.MouseEvent, nodeId: string) => {
      e.preventDefault()
      const node = nodes.find((n) => n.id === nodeId)
      if (!node) return

      const rect = e.currentTarget.getBoundingClientRect()
      setDragOffset({
        x: e.clientX - rect.left,
        y: e.clientY - rect.top,
      })
      setDraggedNode(nodeId)
      setSelectedNode(nodeId)
    },
    [nodes],
  )

  const handleMouseMove = useCallback(
    (e: React.MouseEvent) => {
      if (!draggedNode || !canvasRef.current) return

      const canvasRect = canvasRef.current.getBoundingClientRect()
      const newX = (e.clientX - canvasRect.left - dragOffset.x) / zoom
      const newY = (e.clientY - canvasRect.top - dragOffset.y) / zoom

      setNodes((prevNodes) =>
        prevNodes.map((node) =>
          node.id === draggedNode ? { ...node, x: Math.max(0, newX), y: Math.max(0, newY) } : node,
        ),
      )
    },
    [draggedNode, dragOffset, zoom],
  )

  const handleMouseUp = useCallback(() => {
    setDraggedNode(null)
  }, [])

  const handleZoomIn = () => setZoom((prev) => Math.min(prev + 0.1, 2))
  const handleZoomOut = () => setZoom((prev) => Math.max(prev - 0.1, 0.5))
  const handleResetZoom = () => setZoom(1)

  const renderConnection = (connection: Connection) => {
    const fromNode = nodes.find((n) => n.id === connection.from)
    const toNode = nodes.find((n) => n.id === connection.to)
    if (!fromNode || !toNode) return null

    const fromX = fromNode.x + fromNode.width
    const fromY = fromNode.y + fromNode.height / 2
    const toX = toNode.x
    const toY = toNode.y + toNode.height / 2

    const midX = (fromX + toX) / 2

    return (
      <path
        key={connection.id}
        d={`M ${fromX} ${fromY} C ${midX} ${fromY} ${midX} ${toY} ${toX} ${toY}`}
        stroke="hsl(var(--muted-foreground))"
        strokeWidth="2"
        fill="none"
        markerEnd="url(#arrowhead)"
      />
    )
  }

  return (
    <Card className="h-[600px] overflow-hidden">
      <div className="flex items-center justify-between p-4 border-b border-border">
        <h3 className="text-lg font-semibold">Visual Workflow Editor</h3>
        <div className="flex items-center gap-2">
          <Button variant="outline" size="sm" onClick={handleZoomOut}>
            <ZoomOut className="h-4 w-4" />
          </Button>
          <span className="text-sm text-muted-foreground min-w-[60px] text-center">{Math.round(zoom * 100)}%</span>
          <Button variant="outline" size="sm" onClick={handleZoomIn}>
            <ZoomIn className="h-4 w-4" />
          </Button>
          <Button variant="outline" size="sm" onClick={handleResetZoom}>
            <Maximize className="h-4 w-4" />
          </Button>
          <Button size="sm">
            <Save className="h-4 w-4 mr-2" />
            Save
          </Button>
        </div>
      </div>

      <div
        ref={canvasRef}
        className="relative w-full h-full bg-muted/20 overflow-hidden cursor-grab"
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
      >
        <div
          className="relative w-full h-full"
          style={{
            transform: `scale(${zoom})`,
            transformOrigin: "0 0",
          }}
        >
          {/* Grid background */}
          <div
            className="absolute inset-0 opacity-20"
            style={{
              backgroundImage: `
                linear-gradient(hsl(var(--border)) 1px, transparent 1px),
                linear-gradient(90deg, hsl(var(--border)) 1px, transparent 1px)
              `,
              backgroundSize: "20px 20px",
            }}
          />

          {/* SVG for connections */}
          <svg className="absolute inset-0 w-full h-full pointer-events-none">
            <defs>
              <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
                <polygon points="0 0, 10 3.5, 0 7" fill="hsl(var(--muted-foreground))" />
              </marker>
            </defs>
            {connections.map(renderConnection)}
          </svg>

          {/* Nodes */}
          {nodes.map((node) => (
            <div
              key={node.id}
              className={`absolute border-2 rounded-lg shadow-sm cursor-move transition-all duration-200 ${
                nodeColors[node.type as keyof typeof nodeColors]
              } ${selectedNode === node.id ? "ring-2 ring-accent ring-offset-2" : ""}`}
              style={{
                left: node.x,
                top: node.y,
                width: node.width,
                height: node.height,
              }}
              onMouseDown={(e) => handleNodeMouseDown(e, node.id)}
            >
              <div className="flex items-center justify-center h-full p-2">
                <span className="text-sm font-medium text-white text-center">{node.name}</span>
              </div>

              {/* Input port */}
              {node.type !== "input" && (
                <div className="absolute left-0 top-1/2 transform -translate-x-1/2 -translate-y-1/2 w-3 h-3 bg-background border-2 border-muted-foreground rounded-full" />
              )}

              {/* Output port */}
              {node.type !== "output" && (
                <div className="absolute right-0 top-1/2 transform translate-x-1/2 -translate-y-1/2 w-3 h-3 bg-background border-2 border-muted-foreground rounded-full" />
              )}
            </div>
          ))}
        </div>
      </div>
    </Card>
  )
}
