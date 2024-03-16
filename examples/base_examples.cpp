//
// Created by JS-Robotics on 16.03.24.
//


#include <iostream>
#include "odrive_can_interface/odrive_can_interface.h"

int main(){
  std::cout << "Hello Example" << std::endl;
  ODriveCanInterface drive(0, "can0");
  ODriveCanInterface drive_1(1, "can0");
  ODriveCanInterface drive_2(2, "can0");
}