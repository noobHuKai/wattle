"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Save, Download, Copy } from "lucide-react"

const sampleYaml = `apiVersion: v1
kind: Workflow
metadata:
  name: data-processing-pipeline
  labels:
    app: data-processor
spec:
  entrypoint: main
  templates:
  - name: main
    dag:
      tasks:
      - name: extract-data
        template: extract
      - name: transform-data
        template: transform
        dependencies: [extract-data]
      - name: load-data
        template: load
        dependencies: [transform-data]
  
  - name: extract
    container:
      image: data-extractor:latest
      command: [python]
      args: ["extract.py"]
      
  - name: transform
    container:
      image: data-transformer:latest
      command: [python]
      args: ["transform.py"]
      
  - name: load
    container:
      image: data-loader:latest
      command: [python]
      args: ["load.py"]`

export function WorkflowYamlEditor() {
  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle>Workflow Definition</CardTitle>
          <div className="flex gap-2">
            <Button variant="outline" size="sm">
              <Copy className="h-4 w-4 mr-2" />
              Copy
            </Button>
            <Button variant="outline" size="sm">
              <Download className="h-4 w-4 mr-2" />
              Download
            </Button>
            <Button size="sm">
              <Save className="h-4 w-4 mr-2" />
              Save
            </Button>
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <div className="relative">
          <pre className="bg-muted p-4 rounded-md text-sm font-mono overflow-x-auto max-h-96 overflow-y-auto">
            <code>{sampleYaml}</code>
          </pre>
        </div>
      </CardContent>
    </Card>
  )
}
