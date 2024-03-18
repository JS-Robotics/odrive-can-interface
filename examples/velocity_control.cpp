//
// Created by JS-Robotics on 16.03.24.
//


#include <iostream>

#include "odrive_can_interface/odrive_can_interface.h"
#include "odrive_can_interface/can_interface_manager.h"

int main() {

  CanInterfaceManager &manager = CanInterfaceManager::GetInstance("can0");

  if (!manager.StartCanLink()) {
    return 404;
  }

  std::cout << "Hello Example" << std::endl;

  ODriveCanInterface drive(0, manager);

  drive.SetAxisState(ODriveEnums::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  drive.SetInputVel(20.0,0);

  std::this_thread::sleep_for(std::chrono::milliseconds(4000));

  drive.SetInputVel(0.0,0);
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  drive.SetAxisState(ODriveEnums::AxisState::AXIS_STATE_IDLE);

}