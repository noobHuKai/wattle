#include "wattle-cxx/src/lib.rs.h"  // cxx generated header
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <cstdlib>

int main() {
    std::cout << "🚀 Wattle C++ SDK Demo (Direct cxx bindings)" << std::endl;
    std::cout << "=============================================" << std::endl;

    // Get configuration from environment variables
    const char* workflow_name = std::getenv("WATTLE_WORKFLOW_NAME");
    const char* worker_name = std::getenv("WATTLE_WORKER_NAME");

    if (!workflow_name) workflow_name = "cpp_demo_workflow";
    if (!worker_name) worker_name = "cpp_demo_worker";

    try {
        // Create Wattle client using cxx bindings
        auto client_result = new_wattle_client(workflow_name, worker_name);
        auto client = std::move(client_result);
        
        std::cout << "✅ Created Wattle client for workflow: " << workflow_name << std::endl;

        // 1. Publish JSON data
        std::cout << "📤 Publishing JSON data..." << std::endl;
        std::string json_data = R"({"message": "Hello from C++!", "timestamp": 1693123456, "value": 42.5})";
        
        auto publish_response = publish_json(*client, "demo_service", json_data);
        if (publish_response.success) {
            std::cout << "✅ JSON published successfully" << std::endl;
        } else {
            std::cout << "❌ Failed to publish JSON: " << std::string(publish_response.error) << std::endl;
        }

        // 2. Send a request (will timeout since no other worker is running)
        std::cout << "🔄 Testing request with timeout..." << std::endl;
        std::string request_data = R"({"query": "get_status", "worker_id": "cpp_demo_worker"})";
        
        auto request_response = request_json(*client, "other_worker", "status_service", request_data, 5);
        if (request_response.success) {
            std::cout << "📥 Received response: " << std::string(request_response.data) << std::endl;
        } else {
            std::cout << "⚠️  Request failed (expected): " << std::string(request_response.error) << std::endl;
        }

        // 3. Test subscription
        std::cout << "📡 Setting up subscription..." << std::endl;
        auto subscribe_response = subscribe_json(*client, "notifications");
        if (subscribe_response.success) {
            std::cout << "✅ Subscription started" << std::endl;
            
            // Poll for messages for a few seconds
            std::cout << "🎧 Listening for messages (5 seconds)..." << std::endl;
            auto start_time = std::chrono::steady_clock::now();
            
            while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
                auto message_response = get_next_message(*client);
                if (message_response.success && !message_response.data.empty()) {
                    std::cout << "📨 Received message: " << std::string(message_response.data) << std::endl;
                }
                
                // Small delay to avoid busy waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } else {
            std::cout << "❌ Failed to start subscription: " << std::string(subscribe_response.error) << std::endl;
        }

        // 4. Clean up
        auto cleanup_response = cleanup_client(*client);
        if (cleanup_response.success) {
            std::cout << "✅ Client cleaned up successfully" << std::endl;
        }

        std::cout << "🎉 C++ SDK demo completed!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
