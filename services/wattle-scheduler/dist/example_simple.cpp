#include <iostream>
#include <string>
#include <cstdint>

// Mock implementation for demonstration
struct WattleClient {
    std::string workflow_name;
    std::string worker_name;
};

struct WattleResponse {
    bool success;
    std::string message;
    std::string data;
};

// Simple mock functions
WattleClient* new_wattle_client(const std::string& workflow_name, const std::string& worker_name) {
    return new WattleClient{workflow_name, worker_name};
}

WattleResponse publish_json(WattleClient& client, const std::string& topic, const std::string& data) {
    return {true, "Published to " + topic + " by " + client.workflow_name + "/" + client.worker_name, data};
}

WattleResponse request_json(WattleClient& client, const std::string& worker, const std::string& service, const std::string& data, uint32_t timeout_ms) {
    return {true, "Request to " + worker + "/" + service + " with timeout " + std::to_string(timeout_ms) + "ms", "{\"status\": \"ok\", \"original_data\": " + data + "}"};
}

WattleResponse subscribe_json(WattleClient& client, const std::string& topic) {
    return {true, "Subscribed to " + topic + " by " + client.workflow_name + "/" + client.worker_name, ""};
}

std::string get_workflow_name(const WattleClient& client) {
    return client.workflow_name;
}

std::string get_worker_name(const WattleClient& client) {
    return client.worker_name;
}

void cleanup_client(WattleClient& client) {
    // Nothing to clean up for mock
}

int main() {
    try {
        // 创建客户端
        auto client = new_wattle_client("demo_workflow", "demo_worker");
        
        std::cout << "=== Wattle C++ SDK Demo (Mock Version) ===" << std::endl;
        
        // 获取信息
        std::cout << "Workflow: " << get_workflow_name(*client) << std::endl;
        std::cout << "Worker: " << get_worker_name(*client) << std::endl;
        
        // 发布数据
        auto response1 = publish_json(*client, "data_topic", "{\"message\": \"Hello from C++\"}");
        std::cout << "Publish: " << (response1.success ? "✓" : "✗") 
                  << " " << response1.message << std::endl;
        
        // 发送请求
        auto response2 = request_json(*client, "target_worker", "process_service", 
                                    "{\"data\": \"test_data\"}", 5000);
        std::cout << "Request: " << (response2.success ? "✓" : "✗")
                  << " " << response2.message << std::endl;
        std::cout << "Response data: " << response2.data << std::endl;
        
        // 订阅
        auto response3 = subscribe_json(*client, "notification_topic");
        std::cout << "Subscribe: " << (response3.success ? "✓" : "✗")
                  << " " << response3.message << std::endl;
        
        // 清理
        cleanup_client(*client);
        delete client;
        std::cout << "Client cleaned up successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Demo completed successfully!" << std::endl;
    return 0;
}
