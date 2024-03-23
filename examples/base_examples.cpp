//
// Created by JS-Robotics on 16.03.24.
//

#include "odrive_can_interface/odrive_can_interface.h"
#include "odrive_can_interface/can_interface_manager.h"

int main() {

  // Create a socket CAN interface manager instance
  CanInterfaceManager &manager = CanInterfaceManager::GetInstance("can0");

  // Create an instance of the ODriveCanInterface
  ODriveCanInterface drive(0, manager);

  // Start the socketCAN link
  if (!manager.StartCanLink()) {
    return 404;
  }

  //Now the ODrive interface object can be used
  ODriveFeedback feedback = drive.GetFeedback();
  std::cout << "Bus voltage: " << feedback.bus_voltage << std::endl;
  drive.SetInputVel(12.0, 0.0);

}