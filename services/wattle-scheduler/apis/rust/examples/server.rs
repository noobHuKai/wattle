#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let key_expr = "key/expression";
    let queryable = session.declare_queryable(key_expr).await.unwrap();

    while let Ok(query) = queryable.recv_async().await {
        match query.payload() {
            None => println!(">> [Queryable ] Received Query '{}'", query.selector()),
            Some(query_payload) => {
                // Refer to z_bytes.rs to see how to deserialize different types of message
                let deserialized_payload = query_payload
                    .try_to_string()
                    .unwrap_or_else(|e| e.to_string().into());
                println!(
                    ">> [Queryable ] Received Query '{}' with payload '{}'",
                    query.selector(),
                    deserialized_payload
                )
            }
        }
        // Refer to z_bytes.rs to see how to serialize different types of message
        query
            .reply(key_expr, "Hello")
            .await
            .unwrap_or_else(|e| println!(">> [Queryable ] Error sending reply: {e}"));
    }
}
