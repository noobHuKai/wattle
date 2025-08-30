#include "../include/wattle_client.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdlib>

int main() {
    std::cout << "🚀 Wattle C++ SDK Demo" << std::endl;
    std::cout << "======================" << std::endl;

    // Get configuration from environment variables
    const char* workflow_name = std::getenv("WATTLE_WORKFLOW_NAME");
    const char* worker_name = std::getenv("WATTLE_WORKER_NAME");

    if (!workflow_name) workflow_name = "cpp_demo_workflow";
    if (!worker_name) worker_name = "cpp_demo_worker";

    try {
        // Create Wattle client using RAII wrapper
        wattle::ScopedClient client(workflow_name, worker_name);
        std::cout << "✅ Created Wattle client for workflow: " << workflow_name << std::endl;

        // 1. Publish JSON data
        std::cout << "📤 Publishing JSON data..." << std::endl;
        std::string json_data = R"({"message": "Hello from C++!", "timestamp": 1693123456, "value": 42.5})";
        
        if (client->publishJson("demo_service", json_data)) {
            std::cout << "✅ JSON published successfully" << std::endl;
        } else {
            std::cout << "❌ Failed to publish JSON: " << client->getLastError() << std::endl;
        }

        // 2. Send a request (will timeout since no other worker is running)
        std::cout << "🔄 Testing request with timeout..." << std::endl;
        std::string request_data = R"({"query": "get_status", "worker_id": "cpp_demo_worker"})";
        
        std::string response = client->requestJson("other_worker", "status_service", request_data, 5);
        if (response.empty()) {
            std::cout << "⚠️  Request timed out (expected behavior): " << client->getLastError() << std::endl;
        } else {
            std::cout << "📥 Received response: " << response << std::endl;
        }

        // 3. Test subscription
        std::cout << "📡 Setting up subscription..." << std::endl;
        if (client->subscribeJson("notifications")) {
            std::cout << "✅ Subscription started" << std::endl;
            
            // Poll for messages for a few seconds
            std::cout << "🎧 Listening for messages (5 seconds)..." << std::endl;
            auto start_time = std::chrono::steady_clock::now();
            
            while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
                auto message = client->getNextMessage();
                if (!message.first.empty()) {
                    std::cout << "📨 Received message on " << message.first 
                             << ": " << message.second << std::endl;
                }
                
                // Small delay to avoid busy waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } else {
            std::cout << "❌ Failed to start subscription: " << client->getLastError() << std::endl;
        }

        std::cout << "🎉 C++ SDK demo completed!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
