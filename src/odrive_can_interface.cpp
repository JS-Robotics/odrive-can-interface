//
// Created by JS-Robotics on 16.03.24.
//
#include "odrive_can_interface/odrive_can_interface.h"

void ODriveCanInterface::ExtractEncoderEstimate(const can_frame &frame) {
  {
//    std::cout << "------------------------------------------------------------------------------" << std::endl;
//    std::cout << "Received frame with ID: " << std::hex << frame.can_id;
//    std::cout << " and data: ";
//    for (int k = 0; k < frame.can_dlc; k++) {
//      std::cout << std::hex << static_cast<int>(frame.data[k]) << " ";
//    }
//    memcpy(&position, frame.data, sizeof(float));
//    memcpy(&velocity, frame.data + 4, sizeof(float));
//    std::cout << "Position: " << position << " - Velocity: " << velocity;
//    std::cout << std::endl;
//    std::cout << "------------------------------------------------------------------------------" << std::endl;
    float position;
    float velocity;
    memcpy(&position, frame.data, sizeof(float));
    memcpy(&velocity, frame.data + 4, sizeof(float));
    odrive_feedback_.estimate_position = position;
    odrive_feedback_.estimate_velocity = velocity;

  }
}

void ODriveCanInterface::ExtractMotorError(const can_frame &frame) {

}

void ODriveCanInterface::ExtractEncoderError(const can_frame &frame) {

}

void ODriveCanInterface::ExtractControllerError(const can_frame &frame) {

}

void ODriveCanInterface::ExtractSensorlessError(const can_frame &frame) {

}

void ODriveCanInterface::ExtractEncoderCount(const can_frame &frame) {

}

void ODriveCanInterface::ExtractIq(const can_frame &frame) {

}

void ODriveCanInterface::ExtractSensorlessEstimates(const can_frame &frame) {

}

void ODriveCanInterface::ExtractBusVoltageCurrent(const can_frame &frame) {

}

