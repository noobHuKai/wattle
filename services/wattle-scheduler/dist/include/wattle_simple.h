#pragma once

#include "cxx.h"
#include <memory>
#include <string>

// Forward declarations
struct WattleClient;

// Simple wrapper structs for C++ interface
struct WattleResponse {
    bool success;
    std::string message;
    std::string data;
    
    WattleResponse(bool s = false, const std::string& m = "", const std::string& d = "") 
        : success(s), message(m), data(d) {}
};

// C++ wrapper functions
std::unique_ptr<WattleClient> new_wattle_client(const std::string& workflow_name, const std::string& worker_name);
WattleResponse publish_json(WattleClient& client, const std::string& topic, const std::string& data);
WattleResponse request_json(WattleClient& client, const std::string& worker, const std::string& service, const std::string& data, uint32_t timeout_ms);
WattleResponse subscribe_json(WattleClient& client, const std::string& topic);
std::string get_workflow_name(const WattleClient& client);
std::string get_worker_name(const WattleClient& client);
void cleanup_client(WattleClient& client);
