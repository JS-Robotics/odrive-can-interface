//
// Created by sondre on 17.03.24.
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
#include <fcntl.h>  // For fcntl
#include <thread>
#include "odrive_can_interface/can_id_enums.h"
#include "odrive_can_interface/odrive_enums.h"


int main() {
  int can_id = 0;
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // Create the socket
  if (s < 0) {
    perror("Socket");
    return 1;
  }

  int flags = fcntl(s, F_GETFL, 0);
  if (flags == -1) {
    perror("fcntl(F_GETFL)");
    close(s);
    return false;
  }
  flags |= O_NONBLOCK;
  if (fcntl(s, F_SETFL, flags) == -1) {
    perror("fcntl(F_SETFL)");
    close(s);
    return false;
  }

  struct sockaddr_can addr;
  struct ifreq ifr;
  strcpy(ifr.ifr_name, "can0");
  ioctl(s, SIOCGIFINDEX, &ifr); // Specify the interface you wish to use

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  bind(s, (struct sockaddr *) &addr, sizeof(addr)); // Bind the socket to the network interface


  can_frame frame;

  // Continuous loop to read all received frames
  for(int i = 0; i < 10; i++) {
    int nbytes = read(s, &frame, sizeof(struct can_frame)); // Read a frame
    if (nbytes < 0) {
      perror("Read");
//      break; // Exit the loop if there is a read error
    } else if (nbytes == sizeof(struct can_frame)) {
      std::cout << "Received frame with ID: " << std::hex << frame.can_id;
      std::cout << " and data: ";
      for (int k = 0; k < frame.can_dlc; k++) {
        std::cout << std::hex << static_cast<int>(frame.data[k]) << " ";
      }
      std::cout << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Reduce CPU usage
    // Add some exit condition if necessary, for example:
    // if (frame.can_id == some_specific_id) break;
  }

  close(s); // Close the socket
  return 0;
}
