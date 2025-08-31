"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { TimeRangeSelector } from "./time-range-selector"
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from "recharts"
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

interface SystemMetricsChartProps {
  title: string
  metric: "cpu" | "memory"
  color: string
  unit: string
}

export function SystemMetricsChart({ title, metric, color, unit }: SystemMetricsChartProps) {
  const [timeRange, setTimeRange] = useState("24h")
  const data = generateMockData(timeRange)

  return (
    <Card>
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-base font-semibold">{title}</CardTitle>
        <TimeRangeSelector selected={timeRange} onSelect={setTimeRange} />
      </CardHeader>
      <CardContent>
        <div className="h-[200px]">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
              <XAxis dataKey="time" className="text-xs fill-muted-foreground" tick={{ fontSize: 12 }} />
              <YAxis
                className="text-xs fill-muted-foreground"
                tick={{ fontSize: 12 }}
                domain={["dataMin - 5", "dataMax + 5"]}
              />
              <Tooltip
                contentStyle={{
                  backgroundColor: "hsl(var(--card))",
                  border: "1px solid hsl(var(--border))",
                  borderRadius: "6px",
                }}
                labelStyle={{ color: "hsl(var(--foreground))" }}
              />
              <Line
                type="monotone"
                dataKey={metric}
                stroke={color}
                strokeWidth={2}
                dot={false}
                activeDot={{ r: 4, fill: color }}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
        <div className="mt-2 text-right">
          <span className="text-sm text-muted-foreground">Current: </span>
          <span className="text-sm font-semibold" style={{ color }}>
            {data[data.length - 1]?.[metric]?.toFixed(1)}
            {unit}
          </span>
        </div>
      </CardContent>
    </Card>
  )
}
