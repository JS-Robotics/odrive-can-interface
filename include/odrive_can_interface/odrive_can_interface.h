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
  ODriveCanInterface(int can_id, CanInterfaceManager &manager)
      : can_id_(can_id), manager_(manager) {
    std::cout << "Constructor - can ID: " << can_id_ <<  std::endl;

//    manager_ = &CanInterfaceManager::GetInstance(can_link_);
    // Register this interface with the manager
    manager_.AddODriveInterface(can_id_, this);

  }

  ~ODriveCanInterface() {
    std::cout << "Destructor - can ID: " << can_id_  << std::endl;
  }

  void HandleCanMessage(const can_frame &frame) override {
    std::cout << "HandleCanMessage" << std::endl;
  }

  void SetInputVel(double velocity) {
    // Construct the CAN message for setting velocity.
    can_frame frame;
    // Populate the frame with appropriate ID and data for setting velocity.
    // Note: The specifics here will depend on your ODrive's CAN protocol.

    // Send the message through the CanInterfaceManager.
    manager_.SendCanMessage(frame);
  }

 private:
  const int can_id_ = 0x00;
  CanInterfaceManager &manager_;
};

#endif //ODRIVE_CAN_INTERFACE_ODRIVE_CAN_INTERFACE_H_
