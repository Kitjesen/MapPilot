#pragma once

#include <memory>
#include <cstdint>
#include <string>

namespace lingtu::nav::commands {

class Client {
 public:
  class NavigationCommands {
   public:
    void sendGoal(
        double x,
        double y,
        double z,
        double yaw,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void cancel(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void sendTeleop(
        double vx,
        double vy,
        double wz,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void stop(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void estop(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void clearEstop(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void resumeAutonomy(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});

   private:
    friend class Client;
    explicit NavigationCommands(Client& owner) : owner_(owner) {}
    Client& owner_;
  };

  class InspectionCommands {
   public:
    void start(
        const std::string& route_id,
        std::uint64_t route_revision = 0,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void pause(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void resume(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void cancel(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});

   private:
    friend class Client;
    explicit InspectionCommands(Client& owner) : owner_(owner) {}
    Client& owner_;
  };

  explicit Client(int domain_id);
  ~Client();

  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;
  Client(Client&&) = delete;
  Client& operator=(Client&&) = delete;

  NavigationCommands& navigation() noexcept { return navigation_; }
  InspectionCommands& inspection() noexcept { return inspection_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  NavigationCommands navigation_;
  InspectionCommands inspection_;
};

}  // namespace lingtu::nav::commands
