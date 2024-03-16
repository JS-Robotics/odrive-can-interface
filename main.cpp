//
// Created by JS-Robotics on 16.03.24.
//


#include <unistd.h>
#include <cstring>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>

int main() {
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // Create the socket
  if (s < 0) {
    perror("Socket");
    return 1;
  }

  struct sockaddr_can addr;
  struct ifreq ifr;
  strcpy(ifr.ifr_name, "can0");
  ioctl(s, SIOCGIFINDEX, &ifr); // Specify the interface you wish to use

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  bind(s, (struct sockaddr *)&addr, sizeof(addr)); // Bind the socket to the network interface

  struct can_frame frame;
  frame.can_id = 0x123; // Replace with your ODrive CAN ID
  frame.can_dlc = 2;    // Data length code: number of bytes in the data field
  frame.data[0] = 0x01; // Replace with your data
  frame.data[1] = 0x02; // Replace with your data

  // Send a frame
  if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    perror("Write");
    std::cout << "Write" << s << std::endl;
    return 1;
  }

  // Receive a frame
  if (read(s, &frame, sizeof(struct can_frame)) < 0) {
    perror("Read");
    return 1;
  }

  std::cout << "Received frame with ID: " << std::hex << frame.can_id;
  std::cout << " and data: " << std::hex << (int)frame.data[0];
  std::cout << (int)frame.data[1] << std::endl;

  close(s); // Close the socket
  return 0;
}