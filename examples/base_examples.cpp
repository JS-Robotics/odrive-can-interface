//
// Created by JS-Robotics on 16.03.24.
//


#include <iostream>
#include <csignal>

#include "odrive_can_interface/odrive_can_interface.h"
#include "odrive_can_interface/can_interface_manager.h"

bool shutdown_requested = false;

inline void stop_handler(int) {
  shutdown_requested = true;
  std::cout << "preparing to shut down..." << std::endl;
}

inline void setup_signal_handlers() {
  signal(SIGINT, stop_handler);
  signal(SIGTERM, stop_handler);
}

int main(){
  setup_signal_handlers();

  CanInterfaceManager& manager = CanInterfaceManager::GetInstance("can0");

  std::cout << "Hello Example" << std::endl;
  ODriveCanInterface drive(0, manager);
  ODriveCanInterface drive_1(1, manager);
  ODriveCanInterface drive_2(2, manager);

  while (!shutdown_requested) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    drive.SetInputVel(10.0);
    drive_1.SetInputVel(12.0);
    drive_2.SetInputVel(13.0);
  }

}