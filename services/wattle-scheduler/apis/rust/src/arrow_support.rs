//! Arrow 数据支持

use arrow_array::{Array, ArrayRef, Int32Array, StringArray, RecordBatch};
use arrow_schema::{DataType, Field, Schema};
use serde_json::Value;
use eyre::{Result, eyre};
use std::sync::Arc;

/// 将 RecordBatch 转换为 JSON Value
fn batch_to_json(batch: &RecordBatch) -> Result<Value> {
    let mut rows = Vec::new();
    
    for row in 0..batch.num_rows() {
        let mut row_map = serde_json::Map::new();
        
        for (col_idx, field) in batch.schema().fields().iter().enumerate() {
            let column = batch.column(col_idx);
            let field_name = field.name();
            
            let value = match column.data_type() {
                DataType::Int32 => {
                    let array = column.as_any().downcast_ref::<Int32Array>().unwrap();
                    if array.is_null(row) {
                        Value::Null
                    } else {
                        Value::Number(array.value(row).into())
                    }
                }
                DataType::Utf8 => {
                    let array = column.as_any().downcast_ref::<StringArray>().unwrap();
                    if array.is_null(row) {
                        Value::Null
                    } else {
                        Value::String(array.value(row).to_string())
                    }
                }
                _ => Value::Null, // 简化：不支持的类型设为 null
            };
            
            row_map.insert(field_name.clone(), value);
        }
        
        rows.push(Value::Object(row_map));
    }
    
    Ok(Value::Array(rows))
}

/// 从 JSON Value 创建 RecordBatch
fn json_to_batch(json: &Value) -> Result<RecordBatch> {
    let rows = json.as_array().ok_or_else(|| eyre!("Expected array of rows"))?;
    
    if rows.is_empty() {
        // 创建空的 RecordBatch
        let schema = Arc::new(Schema::new(vec![] as Vec<Field>));
        return Ok(RecordBatch::new_empty(schema));
    }
    
    // 从第一行推断 schema
    let first_row = rows[0].as_object().ok_or_else(|| eyre!("Expected object row"))?;
    let mut fields = Vec::new();
    let mut string_columns: Vec<Vec<Option<String>>> = Vec::new();
    let mut int_columns: Vec<Vec<Option<i32>>> = Vec::new();
    let mut column_types = Vec::new();
    
    for (key, value) in first_row {
        let data_type = match value {
            Value::String(_) => {
                column_types.push("string");
                string_columns.push(Vec::new());
                DataType::Utf8
            }
            Value::Number(n) if n.is_i64() => {
                column_types.push("int");
                int_columns.push(Vec::new());
                DataType::Int32
            }
            _ => {
                column_types.push("string"); // 默认为字符串
                string_columns.push(Vec::new());
                DataType::Utf8
            }
        };
        
        fields.push(Field::new(key, data_type, true));
    }
    
    // 填充数据
    for row in rows {
        let row_obj = row.as_object().ok_or_else(|| eyre!("Expected object row"))?;
        let mut string_col_idx = 0;
        let mut int_col_idx = 0;
        
        for (col_idx, (key, _)) in first_row.iter().enumerate() {
            let value = row_obj.get(key);
            
            match column_types[col_idx] {
                "string" => {
                    let val = match value {
                        Some(Value::String(s)) => Some(s.clone()),
                        Some(v) => Some(v.to_string()),
                        None => None,
                    };
                    string_columns[string_col_idx].push(val);
                    string_col_idx += 1;
                }
                "int" => {
                    let val = match value {
                        Some(Value::Number(n)) => n.as_i64().map(|i| i as i32),
                        _ => None,
                    };
                    int_columns[int_col_idx].push(val);
                    int_col_idx += 1;
                }
                _ => {}
            }
        }
    }
    
    // 构建 RecordBatch
    let mut columns: Vec<ArrayRef> = Vec::new();
    let mut string_col_idx = 0;
    let mut int_col_idx = 0;
    
    for col_type in &column_types {
        match *col_type {
            "string" => {
                let array = StringArray::from(string_columns[string_col_idx].clone());
                columns.push(Arc::new(array));
                string_col_idx += 1;
            }
            "int" => {
                let array = Int32Array::from(int_columns[int_col_idx].clone());
                columns.push(Arc::new(array));
                int_col_idx += 1;
            }
            _ => {}
        }
    }
    
    let schema = Arc::new(Schema::new(fields));
    RecordBatch::try_new(schema, columns)
        .map_err(|e| eyre!("Failed to create RecordBatch: {}", e))
}

/// 将 Arrow RecordBatch 转换为字节  
pub fn arrow_to_bytes(batch: &RecordBatch) -> Result<Vec<u8>> {
    let json_value = batch_to_json(batch)?;
    let json_string = serde_json::to_string(&json_value)?;
    Ok(json_string.into_bytes())
}

/// 从字节转换为 Arrow RecordBatch
pub fn bytes_to_arrow(bytes: &[u8]) -> Result<RecordBatch> {
    let json_str = std::str::from_utf8(bytes)?;
    let json_value: Value = serde_json::from_str(json_str)?;
    json_to_batch(&json_value)
}

/// 创建示例 Arrow 数据
pub fn create_sample_batch() -> Result<RecordBatch> {
    let names = StringArray::from(vec!["Alice", "Bob", "Charlie"]);
    let ages = Int32Array::from(vec![25, 30, 35]);

    RecordBatch::try_from_iter(vec![
        ("name", Arc::new(names) as ArrayRef),
        ("age", Arc::new(ages) as ArrayRef),
    ])
    .map_err(|e| eyre!("Failed to create Arrow batch: {}", e))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arrow_serialization() -> Result<()> {
        let batch = create_sample_batch()?;
        let bytes = arrow_to_bytes(&batch)?;
        let recovered_batch = bytes_to_arrow(&bytes)?;
        
        assert_eq!(batch.num_rows(), recovered_batch.num_rows());
        assert_eq!(batch.num_columns(), recovered_batch.num_columns());
        
        Ok(())
    }
    
    #[test]
    fn test_empty_batch() -> Result<()> {
        let schema = Arc::new(Schema::new(vec![] as Vec<Field>));
        let batch = RecordBatch::new_empty(schema);
        let bytes = arrow_to_bytes(&batch)?;
        let recovered_batch = bytes_to_arrow(&bytes)?;
        
        assert_eq!(batch.num_rows(), recovered_batch.num_rows());
        assert_eq!(batch.num_columns(), recovered_batch.num_columns());
        
        Ok(())
    }
}
