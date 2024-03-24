//
// Created by JS-Robotics on 16.03.24.
//


#include "odrive_can_interface/odrive_can_interface.h"

void ODriveCanInterface::ExtractMotorError(const can_frame &frame) {
  uint64_t motor_error;
  memcpy(&motor_error, frame.data, sizeof(uint64_t));
  odrive_feedback_.error.motor = motor_error;
}

void ODriveCanInterface::ExtractEncoderError(const can_frame &frame) {
  uint32_t encoder_error;
  memcpy(&encoder_error, frame.data, sizeof(uint32_t));
  odrive_feedback_.error.encoder = encoder_error;
}

void ODriveCanInterface::ExtractControllerError(const can_frame &frame) {
  uint32_t controller_error;
  memcpy(&controller_error, frame.data, sizeof(uint32_t));
  odrive_feedback_.error.controller = controller_error;
}

void ODriveCanInterface::ExtractSensorlessError(const can_frame &frame) {
  uint32_t sensorless_error;
  memcpy(&sensorless_error, frame.data, sizeof(uint32_t));
  odrive_feedback_.error.sensorless = sensorless_error;
}

void ODriveCanInterface::ExtractEncoderEstimate(const can_frame &frame) {
  {
    float position;
    float velocity;
    memcpy(&position, frame.data, sizeof(float));
    memcpy(&velocity, frame.data + 4, sizeof(float));
    odrive_feedback_.estimate_position = position;
    odrive_feedback_.estimate_velocity = velocity;
  }
}

void ODriveCanInterface::ExtractEncoderCount(const can_frame &frame) {
  int32_t encoder_count_in_cpr;
  int32_t encoder_shadow_count;
  memcpy(&encoder_count_in_cpr, frame.data, sizeof(int32_t));
  memcpy(&encoder_shadow_count, frame.data + 4, sizeof(int32_t));
  odrive_feedback_.encoder_count_cpr = encoder_count_in_cpr;
  odrive_feedback_.encoder_shadow_count = encoder_shadow_count;
}

void ODriveCanInterface::ExtractIq(const can_frame &frame) {
  float iq_setpoint;
  float iq_measured;
  memcpy(&iq_setpoint, frame.data, sizeof(float));
  memcpy(&iq_measured, frame.data + 4, sizeof(float));
  odrive_feedback_.iq_setpoint = iq_setpoint;
  odrive_feedback_.iq_measured = iq_measured;
}

void ODriveCanInterface::ExtractSensorlessEstimates(const can_frame &frame) {
  float sensorless_pos_estimate;
  float sensorless_vel_estimate;
  memcpy(&sensorless_pos_estimate, frame.data, sizeof(float));
  memcpy(&sensorless_vel_estimate, frame.data + 4, sizeof(float));
  odrive_feedback_.sensorless_pos_estimate = sensorless_pos_estimate;
  odrive_feedback_.sensorless_vel_estimate = sensorless_vel_estimate;
}

void ODriveCanInterface::ExtractBusVoltageCurrent(const can_frame &frame) {
  float bus_voltage;
  float bus_current;
  memcpy(&bus_voltage, frame.data, sizeof(float));
  memcpy(&bus_current, frame.data + 4, sizeof(float));
  odrive_feedback_.bus_voltage = bus_voltage;
  odrive_feedback_.bus_current = bus_current;
}

