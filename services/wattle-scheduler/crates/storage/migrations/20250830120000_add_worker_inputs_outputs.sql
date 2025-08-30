-- Add inputs and outputs columns to workers table
-- Migration: 20250830120000_add_worker_inputs_outputs.sql

ALTER TABLE workers ADD COLUMN inputs TEXT;
ALTER TABLE workers ADD COLUMN outputs TEXT;
