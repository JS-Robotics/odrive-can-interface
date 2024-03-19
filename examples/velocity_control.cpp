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

  ODriveCanInterface drive(3, manager);

  drive.SetAxisState(ODriveEnums::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  drive.SetInputVel(50.0,0);

  ODriveFeedback feedback;
  for (int i = 0; i < 100; i++){
    feedback = drive.GetFeedback();
    std::cout << "Velocity: " << feedback.estimate_velocity << "[rev/s] - Position: " << feedback.estimate_position << "[rev]" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(4000/100));
  }

  drive.SetInputVel(0.0,0);
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  drive.SetAxisState(ODriveEnums::AxisState::AXIS_STATE_IDLE);

}