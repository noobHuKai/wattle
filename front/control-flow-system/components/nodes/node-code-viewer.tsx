"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Download, Copy, Edit } from "lucide-react"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"

const samplePythonCode = `#!/usr/bin/env python3
"""
Data Extractor Node
Extracts data from various sources including databases, APIs, and files.
"""

import json
import logging
from typing import Dict, Any, Optional
from abc import ABC, abstractmethod

class DataExtractor(ABC):
    """Abstract base class for data extractors."""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.logger = logging.getLogger(__name__)
        
    @abstractmethod
    def extract(self) -> Dict[str, Any]:
        """Extract data from the configured source."""
        pass
        
    def validate_config(self) -> bool:
        """Validate the configuration."""
        required_fields = ['source', 'connection']
        return all(field in self.config for field in required_fields)

class DatabaseExtractor(DataExtractor):
    """Extracts data from database sources."""
    
    def extract(self) -> Dict[str, Any]:
        """Extract data from database."""
        if not self.validate_config():
            raise ValueError("Invalid configuration")
            
        connection = self.config['connection']
        host = connection['host']
        port = connection['port']
        
        self.logger.info(f"Connecting to database at {host}:{port}")
        
        # Database connection logic here
        data = {
            "status": "success",
            "records": [],
            "metadata": {
                "source": "database",
                "timestamp": "2024-01-20T14:22:00Z"
            }
        }
        
        return data

def main():
    """Main entry point for the node."""
    config = {
        "source": "database",
        "connection": {
            "host": "localhost",
            "port": 5432,
            "credentials": {
                "username": "user",
                "password": "pass"
            }
        },
        "output": {
            "format": "json",
            "compression": False
        }
    }
    
    extractor = DatabaseExtractor(config)
    result = extractor.extract()
    
    print(json.dumps(result, indent=2))

if __name__ == "__main__":
    main()`

const sampleDockerfile = `FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \\
    gcc \\
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Create non-root user
RUN useradd --create-home --shell /bin/bash app \\
    && chown -R app:app /app
USER app

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \\
    CMD python -c "import requests; requests.get('http://localhost:8080/health')"

# Run the application
CMD ["python", "main.py"]`

const sampleRequirements = `requests==2.31.0
psycopg2-binary==2.9.7
sqlalchemy==2.0.21
pydantic==2.4.2
fastapi==0.104.1
uvicorn==0.24.0
python-dotenv==1.0.0
structlog==23.1.0`

export function NodeCodeViewer() {
  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle>Source Code</CardTitle>
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
              <Edit className="h-4 w-4 mr-2" />
              Edit
            </Button>
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="main" className="w-full">
          <TabsList className="grid w-full grid-cols-4">
            <TabsTrigger value="main">main.py</TabsTrigger>
            <TabsTrigger value="dockerfile">Dockerfile</TabsTrigger>
            <TabsTrigger value="requirements">requirements.txt</TabsTrigger>
            <TabsTrigger value="config">config.json</TabsTrigger>
          </TabsList>

          <TabsContent value="main" className="mt-4">
            <div className="flex items-center gap-2 mb-2">
              <Badge variant="secondary">Python</Badge>
              <span className="text-sm text-muted-foreground">156 lines</span>
            </div>
            <pre className="bg-muted p-4 rounded-md text-sm font-mono overflow-x-auto max-h-96 overflow-y-auto">
              <code className="language-python">{samplePythonCode}</code>
            </pre>
          </TabsContent>

          <TabsContent value="dockerfile" className="mt-4">
            <div className="flex items-center gap-2 mb-2">
              <Badge variant="secondary">Docker</Badge>
              <span className="text-sm text-muted-foreground">28 lines</span>
            </div>
            <pre className="bg-muted p-4 rounded-md text-sm font-mono overflow-x-auto max-h-96 overflow-y-auto">
              <code className="language-dockerfile">{sampleDockerfile}</code>
            </pre>
          </TabsContent>

          <TabsContent value="requirements" className="mt-4">
            <div className="flex items-center gap-2 mb-2">
              <Badge variant="secondary">Text</Badge>
              <span className="text-sm text-muted-foreground">8 lines</span>
            </div>
            <pre className="bg-muted p-4 rounded-md text-sm font-mono overflow-x-auto max-h-96 overflow-y-auto">
              <code>{sampleRequirements}</code>
            </pre>
          </TabsContent>

          <TabsContent value="config" className="mt-4">
            <div className="flex items-center gap-2 mb-2">
              <Badge variant="secondary">JSON</Badge>
              <span className="text-sm text-muted-foreground">12 lines</span>
            </div>
            <pre className="bg-muted p-4 rounded-md text-sm font-mono overflow-x-auto max-h-96 overflow-y-auto">
              <code className="language-json">{`{
  "name": "data-extractor",
  "version": "1.2.0",
  "description": "Extracts data from various sources",
  "author": "Control Flow Team",
  "license": "MIT",
  "runtime": {
    "memory": "512Mi",
    "cpu": "0.5",
    "timeout": 300
  }
}`}</code>
            </pre>
          </TabsContent>
        </Tabs>
      </CardContent>
    </Card>
  )
}
