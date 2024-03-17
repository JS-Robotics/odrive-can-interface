//
// Created by JS-Robotics on 16.03.24.
//


#ifndef ODRIVECANINTERFACE_INCLUDE_CAN_INTERFACE_MANAGER_H_
#define ODRIVECANINTERFACE_INCLUDE_CAN_INTERFACE_MANAGER_H_

#include <string>
#include <unordered_map>
#include <thread>
#include <mutex>

#include "can_message_handler.h"

class CanInterfaceManager {
 public:
  // Delete copy constructor and copy assignment operator
  CanInterfaceManager(const CanInterfaceManager &) = delete;
  CanInterfaceManager &operator=(const CanInterfaceManager &) = delete;

  // Static method for accessing the single instance
  static CanInterfaceManager &GetInstance(const std::string &interface_name) {
    static CanInterfaceManager instance(interface_name);
    return instance;
  }

  void AddODriveInterface(int node_id, CanMessageHandler *interface) {
    odrive_interfaces_[node_id] = interface;
    std::cout << "Added device: " << node_id << std::endl;

  };

  bool StartCanLink(){

  }

  void SendCanMessage(const can_frame& frame){
    std::cout << "WritingCanMessage" << std::endl;
  }

 private:
  explicit CanInterfaceManager(const std::string &interface_name) : interface_name_(interface_name) {
    std::cout << "CanInterfaceManager Constructor with interface name: " << interface_name_ << std::endl;
    read_thread_ = std::thread(&CanInterfaceManager::ReadCanMessages, this);
  };

  ~CanInterfaceManager() {
    std::cout << "CanInterfaceManager Destructor with interface name: " << interface_name_ << std::endl;
    stop_thread_ = true;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
  }

  void DispatchMessage(int node_id, const can_frame& frame) {
    std::lock_guard<std::mutex> guard(map_mutex_);  // Assuming multithreaded access
    auto it = odrive_interfaces_.find(node_id);
    if (it != odrive_interfaces_.end()) {
      it->second->HandleCanMessage(frame);  // Call the message handler for the corresponding ODrive interface.
    } else {
      std::cerr << "No ODriveCanInterface registered for node ID " << node_id << std::endl;
    }
  }

  void ReadCanMessages() {
    while (!stop_thread_) {
      can_frame frame{};
      std::cout << "I am reading messages" << std::endl;

      if (ReadCanFrame(frame)) {

      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }

  bool ReadCanFrame(const can_frame &frame){
    can_frame test = frame;
  }

  std::unordered_map<int, CanMessageHandler *> odrive_interfaces_;
  std::string interface_name_;
  bool stop_thread_ = false;
  std::thread read_thread_;
  std::mutex map_mutex_;
};
#endif //ODRIVECANINTERFACE_INCLUDE_CAN_INTERFACE_MANAGER_H_
