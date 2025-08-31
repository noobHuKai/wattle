"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { TimeRangeSelector } from "./time-range-selector"
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from "recharts"
import { useState } from "react"

// Mock data for demonstration
const generateMockData = (range: string) => {
  const points = range === "1h" ? 12 : range === "6h" ? 24 : range === "24h" ? 48 : 168
  const interval = range === "1h" ? 5 : range === "6h" ? 15 : range === "24h" ? 30 : 60

  return Array.from({ length: points }, (_, i) => {
    const time = new Date(Date.now() - (points - i - 1) * interval * 60000)
    return {
      time: time.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" }),
      cpu: Math.floor(Math.random() * 40) + 30 + Math.sin(i * 0.1) * 10,
      memory: Math.floor(Math.random() * 30) + 50 + Math.cos(i * 0.15) * 15,
    }
  })
}

export function CombinedMetricsChart() {
  const [timeRange, setTimeRange] = useState("24h")
  const data = generateMockData(timeRange)

  return (
    <Card className="col-span-2">
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-base font-semibold">System Metrics</CardTitle>
        <TimeRangeSelector selected={timeRange} onSelect={setTimeRange} />
      </CardHeader>
      <CardContent>
        <div className="h-[300px]">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
              <XAxis dataKey="time" className="text-xs fill-muted-foreground" tick={{ fontSize: 12 }} />
              <YAxis
                className="text-xs fill-muted-foreground"
                tick={{ fontSize: 12 }}
                domain={[0, 100]}
                label={{ value: "Usage (%)", angle: -90, position: "insideLeft" }}
              />
              <Tooltip
                contentStyle={{
                  backgroundColor: "hsl(var(--card))",
                  border: "1px solid hsl(var(--border))",
                  borderRadius: "6px",
                }}
                labelStyle={{ color: "hsl(var(--foreground))" }}
              />
              <Legend />
              <Line
                type="monotone"
                dataKey="cpu"
                stroke="hsl(var(--chart-1))"
                strokeWidth={2}
                dot={false}
                activeDot={{ r: 4, fill: "hsl(var(--chart-1))" }}
                name="CPU Usage"
              />
              <Line
                type="monotone"
                dataKey="memory"
                stroke="hsl(var(--chart-2))"
                strokeWidth={2}
                dot={false}
                activeDot={{ r: 4, fill: "hsl(var(--chart-2))" }}
                name="Memory Usage"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
        <div className="mt-4 flex justify-between text-sm">
          <div className="flex items-center gap-2">
            <div className="w-3 h-3 bg-chart-1 rounded-full"></div>
            <span className="text-muted-foreground">CPU: </span>
            <span className="font-semibold text-chart-1">{data[data.length - 1]?.cpu?.toFixed(1)}%</span>
          </div>
          <div className="flex items-center gap-2">
            <div className="w-3 h-3 bg-chart-2 rounded-full"></div>
            <span className="text-muted-foreground">Memory: </span>
            <span className="font-semibold text-chart-2">{data[data.length - 1]?.memory?.toFixed(1)}%</span>
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
