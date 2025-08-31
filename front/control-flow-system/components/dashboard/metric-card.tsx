import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { cn } from "@/lib/utils"

interface MetricCardProps {
  title: string
  value: string | number
  subtitle?: string
  trend?: "up" | "down" | "neutral"
  className?: string
}

export function MetricCard({ title, value, subtitle, trend, className }: MetricCardProps) {
  return (
    <Card className={cn("", className)}>
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-sm font-medium text-muted-foreground">{title}</CardTitle>
      </CardHeader>
      <CardContent>
        <div className="text-2xl font-bold text-foreground">{value}</div>
        {subtitle && (
          <p
            className={cn(
              "text-xs",
              trend === "up" && "text-chart-4",
              trend === "down" && "text-chart-3",
              trend === "neutral" && "text-muted-foreground",
            )}
          >
            {subtitle}
          </p>
        )}
      </CardContent>
    </Card>
  )
}
