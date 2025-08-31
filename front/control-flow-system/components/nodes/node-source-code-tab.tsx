"use client"

import { useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Upload, Edit, Trash2, FolderPlus, File, Folder, ChevronRight, ChevronDown, Plus } from "lucide-react"
import { DropdownMenu, DropdownMenuContent, DropdownMenuItem, DropdownMenuTrigger } from "@/components/ui/dropdown-menu"

// Mock file tree data
const mockFileTree = [
  {
    name: "src",
    type: "folder",
    children: [
      {
        name: "main.py",
        type: "file",
        content:
          "# Main entry point\ndef main():\n    print('Hello, World!')\n\nif __name__ == '__main__':\n    main()",
      },
      {
        name: "utils.py",
        type: "file",
        content: "# Utility functions\ndef helper_function():\n    return 'Helper result'",
      },
      {
        name: "modules",
        type: "folder",
        children: [
          {
            name: "processor.py",
            type: "file",
            content:
              "# Data processing module\nclass DataProcessor:\n    def process(self, data):\n        return data.upper()",
          },
        ],
      },
    ],
  },
  { name: "requirements.txt", type: "file", content: "requests==2.28.1\nnumpy==1.24.0\npandas==1.5.2" },
  {
    name: "README.md",
    type: "file",
    content: "# Node Documentation\n\nThis node processes data from various sources.",
  },
]

interface FileTreeItemProps {
  item: any
  level: number
  onFileSelect: (file: any) => void
  selectedFile: any
}

function FileTreeItem({ item, level, onFileSelect, selectedFile }: FileTreeItemProps) {
  const [isExpanded, setIsExpanded] = useState(level === 0)

  if (item.type === "folder") {
    return (
      <div>
        <div
          className={`flex items-center gap-2 py-1 px-2 hover:bg-muted rounded cursor-pointer`}
          style={{ paddingLeft: `${level * 16 + 8}px` }}
          onClick={() => setIsExpanded(!isExpanded)}
        >
          {isExpanded ? <ChevronDown className="h-4 w-4" /> : <ChevronRight className="h-4 w-4" />}
          <Folder className="h-4 w-4 text-chart-2" />
          <span className="text-sm">{item.name}</span>
        </div>
        {isExpanded &&
          item.children?.map((child: any, index: number) => (
            <FileTreeItem
              key={index}
              item={child}
              level={level + 1}
              onFileSelect={onFileSelect}
              selectedFile={selectedFile}
            />
          ))}
      </div>
    )
  }

  return (
    <div
      className={`flex items-center gap-2 py-1 px-2 hover:bg-muted rounded cursor-pointer ${
        selectedFile?.name === item.name ? "bg-accent" : ""
      }`}
      style={{ paddingLeft: `${level * 16 + 24}px` }}
      onClick={() => onFileSelect(item)}
    >
      <File className="h-4 w-4 text-chart-1" />
      <span className="text-sm">{item.name}</span>
    </div>
  )
}

export function NodeSourceCodeTab() {
  const [selectedFile, setSelectedFile] = useState(mockFileTree[0].children[0])
  const [isEditing, setIsEditing] = useState(false)
  const [editContent, setEditContent] = useState("")

  const handleFileSelect = (file: any) => {
    setSelectedFile(file)
    setEditContent(file.content)
    setIsEditing(false)
  }

  const handleEdit = () => {
    setIsEditing(true)
    setEditContent(selectedFile?.content || "")
  }

  const handleSave = () => {
    // Save logic would go here
    setIsEditing(false)
    if (selectedFile) {
      selectedFile.content = editContent
    }
  }

  return (
    <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 h-[600px]">
      {/* File Tree */}
      <Card className="lg:col-span-1">
        <CardHeader className="pb-3">
          <div className="flex items-center justify-between">
            <CardTitle className="text-base">File Structure</CardTitle>
            <DropdownMenu>
              <DropdownMenuTrigger asChild>
                <Button variant="ghost" size="icon" className="h-8 w-8">
                  <Plus className="h-4 w-4" />
                </Button>
              </DropdownMenuTrigger>
              <DropdownMenuContent>
                <DropdownMenuItem>
                  <File className="mr-2 h-4 w-4" />
                  New File
                </DropdownMenuItem>
                <DropdownMenuItem>
                  <FolderPlus className="mr-2 h-4 w-4" />
                  New Folder
                </DropdownMenuItem>
                <DropdownMenuItem>
                  <Upload className="mr-2 h-4 w-4" />
                  Upload Files
                </DropdownMenuItem>
              </DropdownMenuContent>
            </DropdownMenu>
          </div>
        </CardHeader>
        <CardContent className="p-0">
          <div className="max-h-[500px] overflow-y-auto">
            {mockFileTree.map((item, index) => (
              <FileTreeItem
                key={index}
                item={item}
                level={0}
                onFileSelect={handleFileSelect}
                selectedFile={selectedFile}
              />
            ))}
          </div>
        </CardContent>
      </Card>

      {/* Code Editor */}
      <Card className="lg:col-span-2">
        <CardHeader className="pb-3">
          <div className="flex items-center justify-between">
            <CardTitle className="text-base flex items-center gap-2">
              <File className="h-4 w-4" />
              {selectedFile?.name || "Select a file"}
            </CardTitle>
            <div className="flex gap-2">
              {!isEditing ? (
                <>
                  <Button variant="outline" size="sm" onClick={handleEdit}>
                    <Edit className="h-4 w-4 mr-2" />
                    Edit
                  </Button>
                  <Button variant="outline" size="sm">
                    <Trash2 className="h-4 w-4 mr-2" />
                    Delete
                  </Button>
                </>
              ) : (
                <>
                  <Button variant="outline" size="sm" onClick={() => setIsEditing(false)}>
                    Cancel
                  </Button>
                  <Button size="sm" onClick={handleSave}>
                    Save
                  </Button>
                </>
              )}
            </div>
          </div>
        </CardHeader>
        <CardContent>
          {selectedFile ? (
            <div className="h-[450px]">
              {isEditing ? (
                <textarea
                  value={editContent}
                  onChange={(e) => setEditContent(e.target.value)}
                  className="w-full h-full p-4 font-mono text-sm bg-muted border rounded resize-none focus:outline-none focus:ring-2 focus:ring-ring"
                  placeholder="Enter your code here..."
                />
              ) : (
                <pre className="w-full h-full p-4 font-mono text-sm bg-muted border rounded overflow-auto">
                  <code>{selectedFile.content}</code>
                </pre>
              )}
            </div>
          ) : (
            <div className="h-[450px] flex items-center justify-center text-muted-foreground">
              Select a file to view its contents
            </div>
          )}
        </CardContent>
      </Card>
    </div>
  )
}
