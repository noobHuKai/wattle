-- Wattle Scheduler Database Schema
-- Combined migration file created on 2025-08-31
-- This file contains all database schema definitions and optimizations

-- ================================
-- Core Tables
-- ================================

-- Create workflows table with name as primary key
CREATE TABLE
    IF NOT EXISTS workflows (
        name TEXT PRIMARY KEY,
        working_dir TEXT,
        status TEXT NOT NULL DEFAULT 'created',
        created_at TEXT NOT NULL DEFAULT (datetime ('now')),
        deleted_at TEXT,
        started_at TEXT,
        completed_at TEXT
    );

-- Create workers table with composite primary key (workflow_name, name)
CREATE TABLE
    IF NOT EXISTS workers (
        workflow_name TEXT NOT NULL,
        name TEXT NOT NULL,
        command TEXT NOT NULL,
        args TEXT DEFAULT '[]',
        working_dir TEXT,
        env_vars TEXT,
        inputs TEXT,
        outputs TEXT,
        status TEXT NOT NULL DEFAULT 'created',
        error_message TEXT,
        created_at TEXT NOT NULL DEFAULT (datetime ('now')),
        deleted_at TEXT,
        started_at TEXT,
        completed_at TEXT,
        PRIMARY KEY (workflow_name, name),
        FOREIGN KEY (workflow_name) REFERENCES workflows (name) ON DELETE CASCADE
    );

-- Create worker_logs table
CREATE TABLE
    IF NOT EXISTS worker_logs (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        workflow_name TEXT NOT NULL,
        worker_name TEXT NOT NULL,
        log_type TEXT NOT NULL CHECK (log_type IN ('stdout', 'stderr')),
        file_path TEXT NOT NULL,
        created_at TEXT NOT NULL DEFAULT (datetime ('now')),
        FOREIGN KEY (workflow_name, worker_name) REFERENCES workers (workflow_name, name) ON DELETE CASCADE
    );

-- ================================
-- Performance Indexes
-- ================================

-- Primary query optimization indexes
CREATE INDEX
    IF NOT EXISTS idx_workflows_status ON workflows (status);

CREATE INDEX
    IF NOT EXISTS idx_workers_workflow_name ON workers (workflow_name);

CREATE INDEX
    IF NOT EXISTS idx_workers_status ON workers (status);

-- Composite indexes for common queries
CREATE INDEX
    IF NOT EXISTS idx_workers_workflow_status ON workers (workflow_name, status);

CREATE INDEX
    IF NOT EXISTS idx_worker_logs_workflow_worker ON worker_logs (workflow_name, worker_name);

-- Time-based query indexes
CREATE INDEX
    IF NOT EXISTS idx_workflows_created_at ON workflows (created_at);

CREATE INDEX
    IF NOT EXISTS idx_workers_created_at ON workers (created_at);
