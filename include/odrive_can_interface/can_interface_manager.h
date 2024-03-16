//
// Created by JS-Robotics on 16.03.24.
//


#ifndef ODRIVECANINTERFACE_INCLUDE_CAN_INTERFACE_MANAGER_H_
#define ODRIVECANINTERFACE_INCLUDE_CAN_INTERFACE_MANAGER_H_

#include <string>
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

  void AddODriveInterface(int node_id, CanMessageHandler *interface){
    std::cout << "Adding device: " << node_id << std::endl;
  };

 private:
  explicit CanInterfaceManager(const std::string &interface_name) : interface_name_(interface_name) {
    std::cout << "CanInterfaceManager Constructor with interface name: " << interface_name_ << std::endl;
  };

  ~CanInterfaceManager() {
    std::cout << "CanInterfaceManager Destructor with interface name: " << interface_name_ << std::endl;
  }

  std::string interface_name_;

};
#endif //ODRIVECANINTERFACE_INCLUDE_CAN_INTERFACE_MANAGER_H_
