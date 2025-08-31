//! Apache Arrow data processing example for wattle-rs
//! 
//! This example demonstrates how to work with Apache Arrow format data
//! including RecordBatch creation, serialization, and processing.

use wattle_rs::{Wattle, DataFormat, WattleResult};
use tokio::time::{sleep, Duration};
use std::sync::Arc;

#[cfg(feature = "arrow")]
use arrow::{
    array::{Int32Array, StringArray, Float64Array},
    datatypes::{DataType, Field, Schema},
    record_batch::RecordBatch,
};

#[tokio::main]
async fn main() -> WattleResult<()> {
    wattle_rs::init_tracing();
    
    #[cfg(not(feature = "arrow"))]
    {
        println!("❌ Arrow feature is not enabled. Please run with --features arrow");
        return Ok(());
    }
    
    #[cfg(feature = "arrow")]
    {
        println!("🚀 Starting Apache Arrow data processing example");
        
        let wattle = Wattle::new().await?;
        
        // Example 1: Creating and publishing Arrow RecordBatch
        println!("\n📊 Example 1: Creating and publishing Arrow data");
        
        // Create a schema
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int32, false),
            Field::new("name", DataType::Utf8, false),
            Field::new("salary", DataType::Float64, true),
            Field::new("department", DataType::Utf8, true),
        ]));
        
        // Create arrays
        let id_array = Arc::new(Int32Array::from(vec![1, 2, 3, 4, 5]));
        let name_array = Arc::new(StringArray::from(vec![
            "Alice", "Bob", "Charlie", "Diana", "Eve"
        ]));
        let salary_array = Arc::new(Float64Array::from(vec![
            Some(75000.0), Some(80000.0), Some(90000.0), Some(85000.0), Some(95000.0)
        ]));
        let department_array = Arc::new(StringArray::from(vec![
            Some("Engineering"), Some("Marketing"), Some("Engineering"), 
            Some("Sales"), Some("Engineering")
        ]));
        
        // Create RecordBatch
        let batch = RecordBatch::try_new(
            schema.clone(),
            vec![id_array, name_array, salary_array, department_array],
        ).unwrap();
        
        println!("✅ Created RecordBatch with {} rows and {} columns", 
                batch.num_rows(), batch.num_columns());
        
        // Subscribe to Arrow data
        wattle.subscribe_async("data/employees", |data| async move {
            match data {
                DataFormat::Arrow(batch) => {
                    println!("📊 Received Arrow RecordBatch:");
                    println!("   Rows: {}", batch.num_rows());
                    println!("   Columns: {}", batch.num_columns());
                    
                    // Process the data (example: count by department)
                    if let Some(dept_column) = batch.column_by_name("department") {
                        if let Some(dept_array) = dept_column.as_any().downcast_ref::<StringArray>() {
                            let mut dept_counts = std::collections::HashMap::new();
                            for i in 0..dept_array.len() {
                                if let Some(dept) = dept_array.value(i) {
                                    *dept_counts.entry(dept.to_string()).or_insert(0) += 1;
                                }
                            }
                            
                            println!("   Department distribution:");
                            for (dept, count) in dept_counts {
                                println!("     {}: {} employees", dept, count);
                            }
                        }
                    }
                }
                _ => println!("📦 Received non-Arrow data: {}", data),
            }
            Ok(())
        }).await?;
        
        // Publish the Arrow data
        let arrow_data = DataFormat::arrow(batch);
        wattle.publish("data/employees", arrow_data).await?;
        
        sleep(Duration::from_millis(200)).await;
        
        // Example 2: Arrow data transformation pipeline
        println!("\n🔄 Example 2: Arrow data transformation pipeline");
        
        // Subscribe to raw data and transform it
        wattle.subscribe_async("pipeline/input", move |data| {
            let wattle_clone = wattle.clone();
            async move {
                match data {
                    DataFormat::Arrow(input_batch) => {
                        println!("🔧 Processing input batch with {} rows", input_batch.num_rows());
                        
                        // Example transformation: filter high-salary employees
                        if let (Some(salary_col), Some(name_col)) = (
                            input_batch.column_by_name("salary"),
                            input_batch.column_by_name("name")
                        ) {
                            if let (Some(salary_array), Some(name_array)) = (
                                salary_col.as_any().downcast_ref::<Float64Array>(),
                                name_col.as_any().downcast_ref::<StringArray>()
                            ) {
                                let mut high_salary_names = Vec::new();
                                let mut high_salaries = Vec::new();
                                
                                for i in 0..salary_array.len() {
                                    if let Some(salary) = salary_array.value(i) {
                                        if salary > 85000.0 {
                                            high_salary_names.push(name_array.value(i));
                                            high_salaries.push(Some(salary));
                                        }
                                    }
                                }
                                
                                if !high_salary_names.is_empty() {
                                    // Create filtered batch
                                    let filtered_schema = Arc::new(Schema::new(vec![
                                        Field::new("name", DataType::Utf8, false),
                                        Field::new("salary", DataType::Float64, false),
                                    ]));
                                    
                                    let filtered_name_array = Arc::new(StringArray::from(high_salary_names));
                                    let filtered_salary_array = Arc::new(Float64Array::from(high_salaries));
                                    
                                    let filtered_batch = RecordBatch::try_new(
                                        filtered_schema,
                                        vec![filtered_name_array, filtered_salary_array],
                                    ).unwrap();
                                    
                                    // Publish transformed data
                                    let transformed_data = DataFormat::arrow(filtered_batch);
                                    wattle_clone.publish("pipeline/output", transformed_data).await?;
                                    
                                    println!("✅ Filtered {} high-salary employees", high_salaries.len());
                                }
                            }
                        }
                    }
                    _ => println!("⚠️  Expected Arrow data in pipeline"),
                }
                Ok(())
            }
        }).await?;
        
        // Subscribe to transformed data
        wattle.subscribe_async("pipeline/output", |data| async move {
            match data {
                DataFormat::Arrow(batch) => {
                    println!("📈 Received filtered data:");
                    
                    if let (Some(name_col), Some(salary_col)) = (
                        batch.column_by_name("name"),
                        batch.column_by_name("salary")
                    ) {
                        if let (Some(name_array), Some(salary_array)) = (
                            name_col.as_any().downcast_ref::<StringArray>(),
                            salary_col.as_any().downcast_ref::<Float64Array>()
                        ) {
                            for i in 0..batch.num_rows() {
                                let name = name_array.value(i);
                                let salary = salary_array.value(i);
                                println!("   💰 {}: ${:.2}", name, salary);
                            }
                        }
                    }
                }
                _ => println!("📦 Received non-Arrow data in output"),
            }
            Ok(())
        }).await?;
        
        // Publish to pipeline
        let pipeline_batch = RecordBatch::try_new(
            schema.clone(),
            vec![
                Arc::new(Int32Array::from(vec![1, 2, 3, 4, 5])),
                Arc::new(StringArray::from(vec![
                    "Alice", "Bob", "Charlie", "Diana", "Eve"
                ])),
                Arc::new(Float64Array::from(vec![
                    Some(75000.0), Some(80000.0), Some(90000.0), Some(85000.0), Some(95000.0)
                ])),
                Arc::new(StringArray::from(vec![
                    Some("Engineering"), Some("Marketing"), Some("Engineering"), 
                    Some("Sales"), Some("Engineering")
                ])),
            ],
        ).unwrap();
        
        wattle.publish("pipeline/input", DataFormat::arrow(pipeline_batch)).await?;
        
        sleep(Duration::from_millis(300)).await;
        
        // Example 3: Arrow data aggregation service
        println!("\n📊 Example 3: Arrow data aggregation service");
        
        wattle.register_request_handler("analytics/aggregate", |request| async move {
            match request.data {
                DataFormat::Arrow(batch) => {
                    println!("📊 Performing aggregation on {} rows", batch.num_rows());
                    
                    let mut result = serde_json::json!({
                        "row_count": batch.num_rows(),
                        "column_count": batch.num_columns(),
                        "columns": []
                    });
                    
                    // Analyze each column
                    for (i, field) in batch.schema().fields().iter().enumerate() {
                        let column = batch.column(i);
                        let mut column_info = serde_json::json!({
                            "name": field.name(),
                            "data_type": format!("{:?}", field.data_type()),
                            "null_count": column.null_count(),
                        });
                        
                        // Add type-specific statistics
                        match field.data_type() {
                            DataType::Float64 => {
                                if let Some(float_array) = column.as_any().downcast_ref::<Float64Array>() {
                                    let mut sum = 0.0;
                                    let mut count = 0;
                                    let mut min = f64::INFINITY;
                                    let mut max = f64::NEG_INFINITY;
                                    
                                    for i in 0..float_array.len() {
                                        if let Some(value) = float_array.value(i) {
                                            sum += value;
                                            count += 1;
                                            min = min.min(value);
                                            max = max.max(value);
                                        }
                                    }
                                    
                                    if count > 0 {
                                        column_info["statistics"] = serde_json::json!({
                                            "sum": sum,
                                            "average": sum / count as f64,
                                            "min": min,
                                            "max": max,
                                        });
                                    }
                                }
                            }
                            DataType::Int32 => {
                                if let Some(int_array) = column.as_any().downcast_ref::<Int32Array>() {
                                    let mut sum = 0i64;
                                    let mut count = 0;
                                    let mut min = i32::MAX;
                                    let mut max = i32::MIN;
                                    
                                    for i in 0..int_array.len() {
                                        let value = int_array.value(i);
                                        sum += value as i64;
                                        count += 1;
                                        min = min.min(value);
                                        max = max.max(value);
                                    }
                                    
                                    if count > 0 {
                                        column_info["statistics"] = serde_json::json!({
                                            "sum": sum,
                                            "average": sum as f64 / count as f64,
                                            "min": min,
                                            "max": max,
                                        });
                                    }
                                }
                            }
                            DataType::Utf8 => {
                                if let Some(string_array) = column.as_any().downcast_ref::<StringArray>() {
                                    let mut unique_values = std::collections::HashSet::new();
                                    let mut total_length = 0;
                                    
                                    for i in 0..string_array.len() {
                                        let value = string_array.value(i);
                                        unique_values.insert(value.to_string());
                                        total_length += value.len();
                                    }
                                    
                                    column_info["statistics"] = serde_json::json!({
                                        "unique_count": unique_values.len(),
                                        "average_length": total_length as f64 / string_array.len() as f64,
                                        "distinct_values": unique_values.into_iter().collect::<Vec<_>>(),
                                    });
                                }
                            }
                            _ => {}
                        }
                        
                        if let Some(columns) = result["columns"].as_array_mut() {
                            columns.push(column_info);
                        }
                    }
                    
                    let response_data = DataFormat::Json(result);
                    Ok(wattle_rs::messaging::Response::success(&request.id, response_data))
                }
                _ => {
                    Ok(wattle_rs::messaging::Response::error(
                        &request.id,
                        "Expected Arrow RecordBatch data"
                    ))
                }
            }
        }).await?;
        
        // Test the aggregation service
        let test_batch = RecordBatch::try_new(
            schema,
            vec![
                Arc::new(Int32Array::from(vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10])),
                Arc::new(StringArray::from(vec![
                    "Alice", "Bob", "Charlie", "Diana", "Eve",
                    "Frank", "Grace", "Henry", "Ivy", "Jack"
                ])),
                Arc::new(Float64Array::from(vec![
                    Some(75000.0), Some(80000.0), Some(90000.0), Some(85000.0), Some(95000.0),
                    Some(70000.0), Some(88000.0), Some(92000.0), Some(78000.0), Some(85000.0)
                ])),
                Arc::new(StringArray::from(vec![
                    Some("Engineering"), Some("Marketing"), Some("Engineering"), 
                    Some("Sales"), Some("Engineering"), Some("Marketing"),
                    Some("Engineering"), Some("Sales"), Some("Marketing"), Some("Engineering")
                ])),
            ],
        ).unwrap();
        
        let response = wattle.request(
            "analytics/aggregate", 
            DataFormat::arrow(test_batch)
        ).await?;
        
        match response.result {
            Ok(DataFormat::Json(stats)) => {
                println!("📈 Aggregation results:");
                println!("{}", serde_json::to_string_pretty(&stats).unwrap());
            }
            Ok(other) => println!("❓ Unexpected response format: {}", other),
            Err(error) => println!("❌ Aggregation failed: {}", error),
        }
        
        // Example 4: Performance metrics
        println!("\n📊 Example 4: Performance metrics");
        
        let metrics = wattle.metrics();
        println!("📈 Arrow processing performance:");
        println!("   Messages published: {}", metrics.messages_published);
        println!("   Messages delivered: {}", metrics.messages_delivered);
        println!("   Requests sent: {}", metrics.requests_sent);
        println!("   Message processing latency: {:?}", 
                Duration::from_micros(metrics.message_processing_latency_us));
        println!("   Request-response latency: {:?}", 
                Duration::from_micros(metrics.request_response_latency_us));
        
        sleep(Duration::from_millis(100)).await;
        
        println!("\n👋 Shutting down Arrow processing example...");
        wattle.shutdown().await?;
        println!("✅ Apache Arrow example completed successfully!");
    }
    
    Ok(())
}
