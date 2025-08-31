"use client"

import { NodeList } from "@/components/nodes/node-list"

export default function NodesPage() {
  return (
    <div className="space-y-6">
      <div className="space-y-2">
        <h1 className="text-3xl font-bold text-foreground">Nodes</h1>
        <p className="text-muted-foreground">Configure and manage your workflow nodes</p>
      </div>

      <NodeList />
    </div>
  )
}
