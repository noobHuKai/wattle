#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let replies = session.get("key/expression").await.unwrap();
    while let Ok(reply) = replies.recv_async().await {
        println!(
            ">> Received {}",
            reply.result().unwrap().payload().try_to_string().unwrap()
        );
    }
}
