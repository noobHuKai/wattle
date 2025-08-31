#include "../include/wattle_client.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <future>
#include <cstdlib>

// Custom callback for handling subscription messages
void custom_message_handler(const std::string& service, const std::string& data) {
    std::cout << "🔔 Custom handler - Service: " << service << ", Data: " << data << std::endl;
}

int main() {
    std::cout << "🚀 Wattle C++ SDK Advanced Demo" << std::endl;
    std::cout << "================================" << std::endl;

    // Configuration
    const char* workflow_name = std::getenv("WATTLE_WORKFLOW_NAME");
    const char* worker_name = std::getenv("WATTLE_WORKER_NAME");

    if (!workflow_name) workflow_name = "cpp_advanced_workflow";
    if (!worker_name) worker_name = "cpp_advanced_worker";

    try {
        wattle::ScopedClient client(workflow_name, worker_name);
        std::cout << "✅ Created advanced Wattle client" << std::endl;

        // 1. Concurrent publishing
        std::cout << "📤 Testing concurrent publishing..." << std::endl;
        
        std::vector<std::future<bool>> publish_futures;
        
        for (int i = 0; i < 5; ++i) {
            publish_futures.push_back(std::async(std::launch::async, [&client, i]() {
                std::string service = "batch_service_" + std::to_string(i);
                std::string data = R"({"batch_id": )" + std::to_string(i) + 
                                  R"(, "data": "concurrent_publish_test", "timestamp": )" + 
                                  std::to_string(std::time(nullptr)) + "}";
                
                return client->publishJson(service, data);
            }));
        }

        // Wait for all publishes to complete
        int successful_publishes = 0;
        for (auto& future : publish_futures) {
            if (future.get()) {
                successful_publishes++;
            }
        }
        
        std::cout << "✅ Completed " << successful_publishes << "/5 concurrent publishes" << std::endl;

        // 2. Multiple subscriptions
        std::cout << "📡 Setting up multiple subscriptions..." << std::endl;
        
        std::vector<std::string> services = {"alerts", "metrics", "logs", "events"};
        
        for (const auto& service : services) {
            if (client->subscribeJson(service)) {
                std::cout << "✅ Subscribed to " << service << std::endl;
            } else {
                std::cout << "❌ Failed to subscribe to " << service 
                         << ": " << client->getLastError() << std::endl;
            }
        }

        // 3. Message processing loop
        std::cout << "🎧 Processing messages for 10 seconds..." << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        int message_count = 0;
        
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
            auto message = client->getNextMessage();
            
            if (!message.first.empty()) {
                message_count++;
                custom_message_handler(message.first, message.second);
                
                // Simulate some processing
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            } else {
                // No message available, short sleep
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        std::cout << "📊 Processed " << message_count << " messages" << std::endl;

        // 4. Batch request testing
        std::cout << "🔄 Testing batch requests..." << std::endl;
        
        std::vector<std::future<std::string>> request_futures;
        
        for (int i = 0; i < 3; ++i) {
            request_futures.push_back(std::async(std::launch::async, [&client, i]() {
                std::string target_worker = "batch_worker_" + std::to_string(i);
                std::string request_data = R"({"request_id": )" + std::to_string(i) + 
                                          R"(, "operation": "batch_process"})";
                
                return client->requestJson(target_worker, "batch_service", request_data, 3);
            }));
        }

        // Process results
        int successful_requests = 0;
        for (auto& future : request_futures) {
            std::string response = future.get();
            if (!response.empty()) {
                successful_requests++;
                std::cout << "📥 Received response: " << response.substr(0, 100) << "..." << std::endl;
            }
        }
        
        std::cout << "✅ Completed " << successful_requests << "/3 batch requests" << std::endl;

        // 5. Error handling demonstration
        std::cout << "🧪 Testing error handling..." << std::endl;
        
        // Try to publish invalid JSON
        if (!client->publishJson("error_test", "invalid_json{")) {
            std::cout << "✅ Correctly caught invalid JSON error: " << client->getLastError() << std::endl;
        }

        // Try to request from non-existent service
        std::string error_response = client->requestJson("non_existent_worker", "fake_service", "{}", 1);
        if (error_response.empty()) {
            std::cout << "✅ Correctly handled non-existent worker: " << client->getLastError() << std::endl;
        }

        std::cout << "🎉 Advanced C++ SDK demo completed!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Critical error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
