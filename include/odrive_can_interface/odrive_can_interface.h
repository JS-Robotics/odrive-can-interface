//
// Created by JS-Robotics on 16.03.24.
//

#ifndef ODRIVE_CAN_INTERFACE_ODRIVE_CAN_INTERFACE_H_
#define ODRIVE_CAN_INTERFACE_ODRIVE_CAN_INTERFACE_H_

#include <unistd.h>
#include <cstring>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>

#include "odrive_can_interface/can_id_enums.h"
#include "odrive_can_interface/odrive_enums.h"
#include "odrive_can_interface/can_interface_manager.h"

class ODriveCanInterface : public CanMessageHandler {
 public:
  ODriveCanInterface(int can_id, const std::string &can_link) : can_id_(can_id), can_link_(can_link) {
    std::cout << "Constructor - can ID: " << can_id_ << " can_link: " << can_link_ << std::endl;

    CanInterfaceManager &manager = CanInterfaceManager::GetInstance(can_link_);
    // Register this interface with the manager
    manager.AddODriveInterface(can_id_, this);

  }

  void HandleCanMessage(const can_frame &frame) override {
    std::cout << "HandleCanMessage" << std::endl;
  }

 private:
  const int can_id_ = 0x00;
  const std::string can_link_;

};

#endif //ODRIVE_CAN_INTERFACE_ODRIVE_CAN_INTERFACE_H_
