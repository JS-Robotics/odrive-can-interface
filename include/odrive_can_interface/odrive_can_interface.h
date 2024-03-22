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

struct ODriveError {
  uint64_t motor{};
  uint32_t encoder{};
  uint32_t controller{};
  uint32_t sensorless{};
};

struct ODriveFeedback {
  ODriveError error;
  float estimate_position{}; ///< The estimated motor position in [rev]
  float estimate_velocity{}; ///< The estimated motor velocity in [rev/s]
  int32_t encoder_count_cpr{}; ///< The current encoder count in [count]
  int32_t encoder_shadow_count{}; ///< [count]
  float iq_setpoint{};
  float iq_measured{};
  float sensorless_pos_estimate{};  ///< [rev]
  float sensorless_vel_estimate{};  ///< [rev/s]
  float bus_voltage{}; ///< [volt]
  float bus_current{}; ///< [ampere]
};

class ODriveCanInterface : public CanMessageHandler {
 public:
  ODriveCanInterface(int can_id, CanInterfaceManager &manager) : can_id_(can_id), manager_(manager) {
    std::cout << "Constructor - can ID: " << can_id_ << std::endl;
    manager_.AddODriveInterface(can_id_, this);
  }

  ~ODriveCanInterface() override {
    std::cout << "Destructor - can ID: " << can_id_ << std::endl;
  }

  [[nodiscard]] ODriveFeedback GetFeedback() const { return odrive_feedback_; }

  void SetAxisState(ODriveEnums::AxisState axis_state) const {
    can_frame state_can_frame{};

    state_can_frame.can_id = can_id_ << 5 | static_cast<int>(ODriveCAN::ID::SET_AXIS_REQUESTED_STATE);
    state_can_frame.can_dlc = 4; // Data length: 4 byte is enough to command the state change
    auto state = static_cast<uint32_t>(axis_state);
    memcpy(state_can_frame.data, &state, sizeof(state));
    manager_.SendCanMessage(state_can_frame);
  }

  void SetControllerModes(ODriveEnums::ControlMode control_mode, ODriveEnums::InputMode input_mode) const {
    can_frame controller_can_frame{};

    controller_can_frame.can_id = can_id_ << 5 | static_cast<int>(ODriveCAN::ID::SET_CONTROLLER_MODES);
    controller_can_frame.can_dlc = 8; // Data length code: 8 bytes. (4 for control mode and 4 for input mode)
    auto control_mode_id = static_cast<int32_t>(control_mode);
    auto input_mode_id = static_cast<int32_t>(input_mode);
    memcpy(controller_can_frame.data, &control_mode_id, sizeof(int32_t));
    memcpy(controller_can_frame.data + 4, &input_mode_id, sizeof(int32_t));
    manager_.SendCanMessage(controller_can_frame);
  }

  void SetInputPos(float position, int16_t feed_forward_velocity, int16_t feed_forward_torque) const {
    can_frame pos_can_frame{};

    pos_can_frame.can_id = (can_id_ << 5) | static_cast<int>(ODriveCAN::ID::SET_INPUT_POS);
    pos_can_frame.can_dlc = 8; // Data length code: 8 bytes
    // (4 for position, 2 for velocity feed-forward and 2 for torque feed-forward)

    feed_forward_velocity = static_cast<int16_t>(feed_forward_velocity * 0.001); //Bus Factor = 0.001
    feed_forward_torque = static_cast<int16_t>(feed_forward_torque * 0.001);  //Bus Factor = 0.001

    memcpy(pos_can_frame.data, &position, sizeof(float));
    memcpy(pos_can_frame.data + 4, &(feed_forward_velocity), sizeof(int16_t));
    memcpy(pos_can_frame.data + 6, &feed_forward_torque, sizeof(int16_t));
    manager_.SendCanMessage(pos_can_frame);
  }

  /*!
   * @param velocity - Velocity input in rev/s
   * @param feed_forward_torque - TODO() figure ut unit
   */
  void SetInputVel(float velocity, float feed_forward_torque) const {
    can_frame vel_can_frame{};

    vel_can_frame.can_id = (can_id_ << 5) | static_cast<int>(ODriveCAN::ID::SET_INPUT_VEL);
    vel_can_frame.can_dlc = 8; // Data length code: 8 bytes (4 for velocity, 4 for torque feed-forward)

    memcpy(vel_can_frame.data, &velocity, sizeof(velocity));
    memcpy(vel_can_frame.data + 4, &feed_forward_torque, sizeof(feed_forward_torque));

    manager_.SendCanMessage(vel_can_frame);
  }

  void SetInputTorque(float torque) {
    can_frame torque_can_frame{};

    torque_can_frame.can_id = (can_id_ << 5) | static_cast<int>(ODriveCAN::ID::SET_INPUT_TORQUE);
    torque_can_frame.can_dlc = 4; // Data length code: 4 bytes (4 for the torque)

    memcpy(torque_can_frame.data, &torque, sizeof(float));

    manager_.SendCanMessage(torque_can_frame);
  }

  void SetLimits(float velocity_limit, float current_limit){
    can_frame limits_can_frame{};

    limits_can_frame.can_id = (can_id_ << 5) | static_cast<int>(ODriveCAN::ID::SET_INPUT_VEL);
    limits_can_frame.can_dlc = 8; // Data length code: 8 bytes (4 for velocity, 4 for torque feed-forward)

    memcpy(limits_can_frame.data, &velocity_limit, sizeof(float));
    memcpy(limits_can_frame.data + 4, &current_limit, sizeof(float));

    manager_.SendCanMessage(limits_can_frame);
  }



 private:
  void HandleCanMessage(const can_frame &frame) override {
    int can_command = GetCommandId(frame.can_id);
    switch (static_cast<ODriveCAN::ID>(can_command)) {
      case ODriveCAN::ID::GET_ENCODER_ESTIMATES:ExtractEncoderEstimate(frame);
        break;
      case ODriveCAN::ID::GET_MOTOR_ERROR:ExtractMotorError(frame);
        break;
      case ODriveCAN::ID::GET_ENCODER_ERROR:ExtractEncoderError(frame);
        break;
      case ODriveCAN::ID::GET_CONTROLLER_ERROR:ExtractControllerError(frame);
        break;
      case ODriveCAN::ID::GET_SENSORLESS_ERROR:ExtractSensorlessError(frame);
        break;
      case ODriveCAN::ID::GET_ENCODER_COUNT:ExtractEncoderCount(frame);
        break;
      case ODriveCAN::ID::GET_IQ:ExtractIq(frame);
        break;
      case ODriveCAN::ID::GET_SENSORLESS_ESTIMATES:ExtractSensorlessEstimates(frame);
        break;
      case ODriveCAN::ID::GET_BUS_VOLTAGE_AND_CURRENT:ExtractBusVoltageCurrent(frame);
        break;
      default:break;
    }
  }

  void ExtractEncoderEstimate(const can_frame &frame);
  void ExtractMotorError(const can_frame &frame);
  void ExtractEncoderError(const can_frame &frame);
  void ExtractControllerError(const can_frame &frame);
  void ExtractSensorlessError(const can_frame &frame);
  void ExtractEncoderCount(const can_frame &frame);
  void ExtractIq(const can_frame &frame);
  void ExtractSensorlessEstimates(const can_frame &frame);
  void ExtractBusVoltageCurrent(const can_frame &frame);

  static int GetCommandId(canid_t can_id) {
    return static_cast<int>(can_id) & 0x1F;  // 0x1F (in binary: 0001 1111) masks to get command id
  }

  CanInterfaceManager &manager_;
  const int can_id_{};
  ODriveFeedback odrive_feedback_{};
};

#endif //ODRIVE_CAN_INTERFACE_ODRIVE_CAN_INTERFACE_H_
