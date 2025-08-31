import { createFileRoute } from '@tanstack/react-router'
import { MetricCard } from '@/components/dashboard/metric-card'
import { CombinedMetricsChart } from '@/components/dashboard/combined-metrics-chart'
import { SystemOverview } from '@/components/dashboard/system-overview'

function Dashboard() {
  return (
    <div className="space-y-6">
      <div className="space-y-2">
        <h1 className="text-3xl font-bold text-foreground">Dashboard</h1>
        <p className="text-muted-foreground">Monitor your control flow system performance and metrics</p>
      </div>

      <SystemOverview />

      <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
        <MetricCard title="Total Workflows" value={12} subtitle="+2 from last week" trend="up" />
        <MetricCard title="Active Nodes" value={48} subtitle="+5 from last week" trend="up" />
        <MetricCard title="Failed Jobs" value={3} subtitle="-1 from last week" trend="down" />
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <CombinedMetricsChart />
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <div className="bg-card p-6 rounded-lg border border-border">
          <h3 className="text-lg font-semibold text-foreground mb-4">Recent Activity</h3>
          <div className="space-y-3">
            <div className="flex items-center justify-between py-2 border-b border-border last:border-b-0">
              <div>
                <p className="text-sm font-medium text-foreground">Workflow "Data Processing" completed</p>
                <p className="text-xs text-muted-foreground">2 minutes ago</p>
              </div>
              <div className="w-2 h-2 bg-chart-4 rounded-full"></div>
            </div>
            <div className="flex items-center justify-between py-2 border-b border-border last:border-b-0">
              <div>
                <p className="text-sm font-medium text-foreground">Node "API Gateway" restarted</p>
                <p className="text-xs text-muted-foreground">5 minutes ago</p>
              </div>
              <div className="w-2 h-2 bg-chart-2 rounded-full"></div>
            </div>
            <div className="flex items-center justify-between py-2 border-b border-border last:border-b-0">
              <div>
                <p className="text-sm font-medium text-foreground">Worker pool scaled to 8 instances</p>
                <p className="text-xs text-muted-foreground">12 minutes ago</p>
              </div>
              <div className="w-2 h-2 bg-chart-1 rounded-full"></div>
            </div>
          </div>
        </div>

        <div className="bg-card p-6 rounded-lg border border-border">
          <h3 className="text-lg font-semibold text-foreground mb-4">System Health</h3>
          <div className="space-y-4">
            <div className="flex items-center justify-between">
              <span className="text-sm text-muted-foreground">API Response Time</span>
              <span className="text-sm font-medium text-chart-4">142ms</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-sm text-muted-foreground">Database Connections</span>
              <span className="text-sm font-medium text-foreground">24/50</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-sm text-muted-foreground">Queue Length</span>
              <span className="text-sm font-medium text-foreground">7</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-sm text-muted-foreground">Error Rate</span>
              <span className="text-sm font-medium text-chart-3">0.02%</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

export const Route = createFileRoute('/_app/dashboard')({
  component: Dashboard,
})
