"use client"

import { cn } from "@/lib/utils"
import { Button } from "@/components/ui/button"
import { GitBranch, Box, Home } from "lucide-react"
import { Link, useLocation } from '@tanstack/react-router'

const navigation = [
  {
    name: "Dashboard",
    href: "/dashboard",
    icon: Home,
  },
  {
    name: "Workflows",
    href: "/workflows",
    icon: GitBranch,
  },
  {
    name: "Nodes",
    href: "/nodes",
    icon: Box,
  },
]

export function Sidebar() {
  const location = useLocation()
  const pathname = location.pathname

  return (
    <div className="w-64 bg-sidebar border-r border-sidebar-border">
      <div className="flex h-full flex-col">
        <div className="flex-1 flex flex-col pt-6 pb-4 overflow-y-auto">
          <nav className="mt-5 flex-1 px-4 space-y-1">
            {navigation.map((item) => {
              const isActive = pathname === item.href
              return (
                <Link key={item.name} to={item.href as any}>
                  <Button
                    variant={isActive ? "secondary" : "ghost"}
                    className={cn(
                      "w-full justify-start gap-3 h-10",
                      isActive
                        ? "bg-sidebar-accent text-sidebar-accent-foreground"
                        : "text-sidebar-foreground hover:bg-sidebar-accent hover:text-sidebar-accent-foreground",
                    )}
                  >
                    <item.icon className="h-4 w-4" />
                    {item.name}
                  </Button>
                </Link>
              )
            })}
          </nav>
        </div>
      </div>
    </div>
  )
}
