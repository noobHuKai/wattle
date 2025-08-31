"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { ChevronRight, ChevronDown, File, Folder, FolderOpen } from "lucide-react"
import { useState } from "react"

interface TreeNode {
  name: string
  type: "file" | "folder"
  children?: TreeNode[]
  size?: string
  modified?: string
}

const mockFileTree: TreeNode = {
  name: "data-extractor",
  type: "folder",
  children: [
    {
      name: "src",
      type: "folder",
      children: [
        { name: "main.py", type: "file", size: "4.2 KB", modified: "2024-01-20" },
        { name: "extractor.py", type: "file", size: "2.8 KB", modified: "2024-01-19" },
        { name: "config.py", type: "file", size: "1.1 KB", modified: "2024-01-18" },
        {
          name: "utils",
          type: "folder",
          children: [
            { name: "logger.py", type: "file", size: "856 B", modified: "2024-01-15" },
            { name: "validators.py", type: "file", size: "1.3 KB", modified: "2024-01-16" },
          ],
        },
      ],
    },
    {
      name: "tests",
      type: "folder",
      children: [
        { name: "test_extractor.py", type: "file", size: "3.1 KB", modified: "2024-01-20" },
        { name: "test_config.py", type: "file", size: "1.8 KB", modified: "2024-01-19" },
      ],
    },
    { name: "Dockerfile", type: "file", size: "892 B", modified: "2024-01-18" },
    { name: "requirements.txt", type: "file", size: "234 B", modified: "2024-01-17" },
    { name: "README.md", type: "file", size: "1.5 KB", modified: "2024-01-20" },
    { name: ".gitignore", type: "file", size: "156 B", modified: "2024-01-15" },
  ],
}

interface TreeItemProps {
  node: TreeNode
  level: number
}

function TreeItem({ node, level }: TreeItemProps) {
  const [isExpanded, setIsExpanded] = useState(level < 2)

  const handleToggle = () => {
    if (node.type === "folder") {
      setIsExpanded(!isExpanded)
    }
  }

  return (
    <div>
      <div
        className="flex items-center gap-2 py-1 px-2 hover:bg-muted/50 rounded cursor-pointer"
        style={{ paddingLeft: `${level * 20 + 8}px` }}
        onClick={handleToggle}
      >
        {node.type === "folder" ? (
          <>
            {isExpanded ? (
              <ChevronDown className="h-4 w-4 text-muted-foreground" />
            ) : (
              <ChevronRight className="h-4 w-4 text-muted-foreground" />
            )}
            {isExpanded ? <FolderOpen className="h-4 w-4 text-chart-2" /> : <Folder className="h-4 w-4 text-chart-2" />}
          </>
        ) : (
          <>
            <div className="w-4" />
            <File className="h-4 w-4 text-muted-foreground" />
          </>
        )}
        <span className="text-sm text-foreground">{node.name}</span>
        {node.size && <span className="text-xs text-muted-foreground ml-auto">{node.size}</span>}
      </div>
      {node.type === "folder" && isExpanded && node.children && (
        <div>
          {node.children.map((child, index) => (
            <TreeItem key={index} node={child} level={level + 1} />
          ))}
        </div>
      )}
    </div>
  )
}

export function NodeDirectoryTree() {
  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle>Directory Structure</CardTitle>
          <Button variant="outline" size="sm">
            Refresh
          </Button>
        </div>
      </CardHeader>
      <CardContent>
        <div className="max-h-96 overflow-y-auto">
          <TreeItem node={mockFileTree} level={0} />
        </div>
      </CardContent>
    </Card>
  )
}
