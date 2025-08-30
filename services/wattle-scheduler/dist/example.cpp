#include "wattle.h"
#include <iostream>
#include <memory>

int main() {
    try {
        // 创建客户端
        auto client = new_wattle_client("demo_workflow", "demo_worker");
        
        std::cout << "=== Wattle C++ SDK Demo ===" << std::endl;
        
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
        std::cout << "Client cleaned up successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Demo completed successfully!" << std::endl;
    return 0;
}
