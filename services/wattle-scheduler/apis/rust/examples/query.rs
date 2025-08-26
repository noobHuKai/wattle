#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let replies = session.get("@/*/router/subscriber/**").await.unwrap();
    while let Ok(reply) = replies.recv_async().await {
        let sample = reply.result().unwrap();
        println!("Received sample: {}", sample.key_expr());
        println!("Payload: {}", sample.payload().try_to_string().unwrap());
    }
}
