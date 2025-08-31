"use client"

import { WorkflowList } from "@/components/workflows/workflow-list"

export default function WorkflowsPage() {
  return (
    <div className="space-y-6">
      <div className="space-y-2">
        <h1 className="text-3xl font-bold text-foreground">Workflows</h1>
        <p className="text-muted-foreground">Manage and monitor your control flow workflows</p>
      </div>

      <WorkflowList />
    </div>
  )
}
