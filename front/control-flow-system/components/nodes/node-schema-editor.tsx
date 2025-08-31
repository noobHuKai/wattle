"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Save, Download, Copy, RefreshCw } from "lucide-react"

const sampleSchema = `{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "title": "Data Extractor Node",
  "description": "Extracts data from various sources",
  "properties": {
    "input": {
      "type": "object",
      "properties": {
        "source": {
          "type": "string",
          "enum": ["database", "api", "file"],
          "description": "Data source type"
        },
        "connection": {
          "type": "object",
          "properties": {
            "host": {
              "type": "string",
              "description": "Connection host"
            },
            "port": {
              "type": "integer",
              "minimum": 1,
              "maximum": 65535
            },
            "credentials": {
              "type": "object",
              "properties": {
                "username": { "type": "string" },
                "password": { "type": "string" }
              },
              "required": ["username", "password"]
            }
          },
          "required": ["host", "port"]
        }
      },
      "required": ["source", "connection"]
    },
    "output": {
      "type": "object",
      "properties": {
        "format": {
          "type": "string",
          "enum": ["json", "csv", "xml"],
          "default": "json"
        },
        "compression": {
          "type": "boolean",
          "default": false
        }
      }
    },
    "config": {
      "type": "object",
      "properties": {
        "timeout": {
          "type": "integer",
          "minimum": 1000,
          "default": 30000,
          "description": "Timeout in milliseconds"
        },
        "retries": {
          "type": "integer",
          "minimum": 0,
          "maximum": 5,
          "default": 3
        }
      }
    }
  },
  "required": ["input", "output"]
}`

export function NodeSchemaEditor() {
  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle>JSON Schema</CardTitle>
          <div className="flex gap-2">
            <Button variant="outline" size="sm">
              <RefreshCw className="h-4 w-4 mr-2" />
              Validate
            </Button>
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
            <code className="language-json">{sampleSchema}</code>
          </pre>
        </div>
      </CardContent>
    </Card>
  )
}
