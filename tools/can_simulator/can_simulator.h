//
// Created by JS-Robotics on 23.03.24.
//

#ifndef ODRIVECANINTERFACE_TOOLS_CAN_SIMULATOR_CAN_SIMULATOR_H_
#define ODRIVECANINTERFACE_TOOLS_CAN_SIMULATOR_CAN_SIMULATOR_H_

#include <unistd.h>
#include <cstring>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <iostream>

class CanSimulator {
 public:
  CanSimulator() {
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
      perror("Socket");
    }

    // Locate the interface you wish to use
    ifreq ifr{};
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    // Bind the socket to the CAN interface
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int bind_error = bind(socket_, (struct sockaddr *) &addr, sizeof(addr));
    if (bind_error > 0) {
      std::cout << "VCAN bind error: " << bind_error << std::endl;
    }
  }

  ~CanSimulator() {
    close(socket_);
  };

  void SendToBus() {
    can_frame frame{};
    frame.can_id = 0x09;
    frame.can_dlc = 8;
    float vel = 12.2;
    float pos = 20.4;
    memcpy(frame.data, &pos, sizeof(float));
    memcpy(frame.data + 4, &vel, sizeof(float));
    write(socket_, &frame, sizeof(frame));
  }

 private:
  int socket_;
};

#endif //ODRIVECANINTERFACE_TOOLS_CAN_SIMULATOR_CAN_SIMULATOR_H_
