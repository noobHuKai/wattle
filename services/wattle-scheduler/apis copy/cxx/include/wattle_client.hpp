#pragma once
#include <string>
#include <memory>
#include <functional>

// Forward declarations from cxx bridge
struct WattleError;
struct WattleResponse;
class WattleClient;

namespace wattle {

/**
 * @brief C++ interface for Wattle Worker-to-Worker communication
 * 
 * This class provides a C++ wrapper around the Rust Wattle SDK,
 * enabling seamless integration with C++ applications.
 */
class Client {
private:
    std::unique_ptr<WattleClient> client_;

public:
    /**
     * @brief Construct a new Wattle Client
     * 
     * @param workflow_name Name of the workflow
     * @param worker_name Name of this worker
     * @throws std::runtime_error if client creation fails
     */
    Client(const std::string& workflow_name, const std::string& worker_name);

    /**
     * @brief Destroy the Client object and clean up resources
     */
    ~Client();

    // Disable copy constructor and assignment
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    // Enable move constructor and assignment
    Client(Client&& other) noexcept;
    Client& operator=(Client&& other) noexcept;

    /**
     * @brief Publish JSON data to a service
     * 
     * @param service_name Name of the service to publish to
     * @param json_data JSON data as string
     * @return true if successful, false otherwise
     */
    bool publishJson(const std::string& service_name, const std::string& json_data);

    /**
     * @brief Send a JSON request and wait for reply
     * 
     * @param target_worker Name of the target worker
     * @param service_name Name of the service
     * @param json_data JSON data as string
     * @param timeout_secs Timeout in seconds (0 for no timeout)
     * @return Response data as JSON string, empty on error
     */
    std::string requestJson(const std::string& target_worker, 
                           const std::string& service_name,
                           const std::string& json_data, 
                           uint32_t timeout_secs = 30);

    /**
     * @brief Subscribe to JSON messages from a service
     * 
     * @param service_name Name of the service to subscribe to
     * @return true if subscription started successfully
     */
    bool subscribeJson(const std::string& service_name);

    /**
     * @brief Get the next message from subscription queue (polling)
     * 
     * @return Pair of (service_name, json_data), empty strings if no message
     */
    std::pair<std::string, std::string> getNextMessage();

    /**
     * @brief Check if there are any error messages from the last operation
     * 
     * @return Error message, empty if no error
     */
    std::string getLastError() const;

private:
    mutable std::string last_error_;
};

/**
 * @brief RAII wrapper for automatic client cleanup
 */
class ScopedClient {
private:
    std::unique_ptr<Client> client_;

public:
    ScopedClient(const std::string& workflow_name, const std::string& worker_name)
        : client_(std::make_unique<Client>(workflow_name, worker_name)) {}

    Client* operator->() { return client_.get(); }
    const Client* operator->() const { return client_.get(); }
    Client& operator*() { return *client_; }
    const Client& operator*() const { return *client_; }

    Client* get() { return client_.get(); }
    const Client* get() const { return client_.get(); }
};

} // namespace wattle

// C callback function for subscription messages
// This will be called from Rust when messages are received
void handle_subscription_message(const std::string& service_name, const std::string& json_data);
