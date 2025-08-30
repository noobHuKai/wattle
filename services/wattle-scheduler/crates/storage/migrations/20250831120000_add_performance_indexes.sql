-- Add indexes for performance optimization
CREATE INDEX idx_workers_workflow_name ON workers(workflow_name);
CREATE INDEX idx_workers_status ON workers(status);
CREATE INDEX idx_workflows_status ON workflows(status);
