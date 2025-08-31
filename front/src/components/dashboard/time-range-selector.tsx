"use client"

import { Button } from "@/components/ui/button"
import { cn } from "@/lib/utils"

interface TimeRangeSelectorProps {
  selected: string
  onSelect: (range: string) => void
  className?: string
}

const timeRanges = [
  { label: "1H", value: "1h" },
  { label: "6H", value: "6h" },
  { label: "24H", value: "24h" },
  { label: "7D", value: "7d" },
  { label: "30D", value: "30d" },
]

export function TimeRangeSelector({ selected, onSelect, className }: TimeRangeSelectorProps) {
  return (
    <div className={cn("flex gap-1", className)}>
      {timeRanges.map((range) => (
        <Button
          key={range.value}
          variant={selected === range.value ? "secondary" : "ghost"}
          size="sm"
          onClick={() => onSelect(range.value)}
          className="h-8 px-3"
        >
          {range.label}
        </Button>
      ))}
    </div>
  )
}
