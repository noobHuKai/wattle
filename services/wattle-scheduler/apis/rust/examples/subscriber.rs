use wattle_rs::Wattle;

#[tokio::main]
async fn main() {
    unsafe {
        std::env::set_var("WATTLE_TASK_NAME", "example_task");
        std::env::set_var("WATTLE_GROUP_NAME", "example_group");
    }
    let wattle = Wattle::new()
        .await
        .expect("Failed to create Wattle instance");

    wattle
        .subscribe_json("aaa", |msg| {
            println!("Received message: {}", msg);
        })
        .await
        .unwrap();
}
