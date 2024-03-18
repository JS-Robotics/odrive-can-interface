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
    std::cout << "------------------------------------------------------------------------------" << std::endl;
    std::cout << "Received frame with ID: " << std::hex << frame.can_id;
    std::cout << " and data: ";
    for (int k = 0; k < frame.can_dlc; k++) {
      std::cout << std::hex << static_cast<int>(frame.data[k]) << " ";
    }
    std::cout << std::endl;
    std::cout << "------------------------------------------------------------------------------" << std::endl;
  }

  void SetAxisState(ODriveEnums::AxisState axis_state){
    can_frame state_can_frame{};

    state_can_frame.can_id = can_id_ << 5 |  static_cast<int>(ODriveCAN::ID::SET_AXIS_REQUESTED_STATE);
    state_can_frame.can_dlc = 4;    // Data length: 4 byte is enough to command the state change
    auto state = static_cast<uint32_t>(axis_state);; // Assuming 8 is the value for AXIS_STATE_CLOSED_LOOP_CONTROL
    memcpy(state_can_frame.data, &state, sizeof(state)); // This ensures all 4 bytes are set correctly
    manager_.SendCanMessage(state_can_frame);
  }


  /*!
   * @param velocity - Velocity input in rev/s
   * @param feed_forward_torque - TODO() figure ut unit
   */
  void SetInputVel(float velocity, float feed_forward_torque) {
    can_frame vel_can_frame{};

    vel_can_frame.can_id = (can_id_ << 5) | static_cast<int>(ODriveCAN::ID::SET_INPUT_VEL);
    vel_can_frame.can_dlc = 8; // Data length code: 8 bytes (4 for velocity, 4 for torque feed-forward)

    memcpy(vel_can_frame.data, &velocity, sizeof(velocity));
    memcpy(vel_can_frame.data + 4, &feed_forward_torque, sizeof(feed_forward_torque));
    // Send the message through the CanInterfaceManager.
    manager_.SendCanMessage(vel_can_frame);
  }

 private:
  const int can_id_ = 0x00;
  CanInterfaceManager &manager_;
};

#endif //ODRIVE_CAN_INTERFACE_ODRIVE_CAN_INTERFACE_H_
