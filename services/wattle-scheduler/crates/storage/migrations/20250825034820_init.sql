-- Create task_groups table with name as primary key
CREATE TABLE
    IF NOT EXISTS task_groups (
        name TEXT PRIMARY KEY,
        working_dir TEXT,
        status TEXT NOT NULL DEFAULT 'created',
        created_at TEXT NOT NULL DEFAULT (datetime ('now')),
        deleted_at TEXT,
        started_at TEXT,
        completed_at TEXT
    );

-- Create tasks table with composite primary key (group_name, name)
CREATE TABLE
    IF NOT EXISTS tasks (
        group_name TEXT NOT NULL,
        name TEXT NOT NULL,
        command TEXT NOT NULL,
        args TEXT DEFAULT '[]',
        working_dir TEXT,
        env_vars TEXT,
        status TEXT NOT NULL DEFAULT 'created',
        error_message TEXT,
        created_at TEXT NOT NULL DEFAULT (datetime ('now')),
        deleted_at TEXT,
        started_at TEXT,
        completed_at TEXT,
        PRIMARY KEY (group_name, name),
        FOREIGN KEY (group_name) REFERENCES task_groups (name) ON DELETE CASCADE
    );

-- Create task_logs table with group_name + task_name reference
CREATE TABLE
    IF NOT EXISTS task_logs (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        group_name TEXT NOT NULL,
        task_name TEXT NOT NULL,
        log_type TEXT NOT NULL CHECK (log_type IN ('stdout', 'stderr')),
        file_path TEXT NOT NULL,
        created_at TEXT NOT NULL DEFAULT (datetime ('now')),
        FOREIGN KEY (group_name, task_name) REFERENCES tasks (group_name, name) ON DELETE CASCADE
    );