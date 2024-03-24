//
// Created by JS-Robotics on 16.03.24.
//

#ifndef ODRIVECANINTERFACE_INCLUDE_ODRIVE_CAN_INTERFACE_CAN_MESSAGE_HANDLER_H_
#define ODRIVECANINTERFACE_INCLUDE_ODRIVE_CAN_INTERFACE_CAN_MESSAGE_HANDLER_H_

#include <linux/can.h>

class CanMessageHandler {
 public:
  virtual void HandleCanMessage(const can_frame& frame) = 0;
  virtual ~CanMessageHandler() = default;
};

#endif //ODRIVECANINTERFACE_INCLUDE_ODRIVE_CAN_INTERFACE_CAN_MESSAGE_HANDLER_H_
