"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Activity, Cpu, HardDrive, Users } from "lucide-react"

const systemStats = [
  {
    title: "System Status",
    value: "Healthy",
    icon: Activity,
    color: "text-chart-4",
    bgColor: "bg-chart-4/10",
  },
  {
    title: "CPU Usage",
    value: "67.2%",
    icon: Cpu,
    color: "text-chart-1",
    bgColor: "bg-chart-1/10",
  },
  {
    title: "Memory Usage",
    value: "58.4%",
    icon: HardDrive,
    color: "text-chart-2",
    bgColor: "bg-chart-2/10",
  },
  {
    title: "Active Workers",
    value: "8",
    icon: Users,
    color: "text-chart-5",
    bgColor: "bg-chart-5/10",
  },
]

export function SystemOverview() {
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
      {systemStats.map((stat) => (
        <Card key={stat.title}>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium text-muted-foreground">{stat.title}</CardTitle>
            <div className={`p-2 rounded-md ${stat.bgColor}`}>
              <stat.icon className={`h-4 w-4 ${stat.color}`} />
            </div>
          </CardHeader>
          <CardContent>
            <div className={`text-2xl font-bold ${stat.color}`}>{stat.value}</div>
          </CardContent>
        </Card>
      ))}
    </div>
  )
}
