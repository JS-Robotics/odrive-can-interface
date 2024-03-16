//
// Created by JS-Robotics on 16.03.24.
//

#ifndef HIVE_CORE_LIBRARIES_INCLUDE_ODRIVE_CAN_INTERFACE_ODRIVE_CAN_INTERFACE_H_
#define HIVE_CORE_LIBRARIES_INCLUDE_ODRIVE_CAN_INTERFACE_ODRIVE_CAN_INTERFACE_H_

#include <unistd.h>
#include <cstring>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>

class ODriveCanInterface {
 public:
  ODriveCanInterface() {
    std::cout << "Constructor" << std::endl;
    int value = DevTest();
    std::cout << "DevTest returned: " << value << std::endl;
  }

  int DevTest() {
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

    bind(s, (struct sockaddr *) &addr, sizeof(addr)); // Bind the socket to the network interface

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
//    if (read(s, &frame, sizeof(struct can_frame)) < 0) {
//      perror("Read");
//      return 1;
//    }

    can_frame setStateFrame;
    setStateFrame.can_id = 0x00 << 5 | 0x07; // Example CAN ID for setting axis state, adjust as necessary
    setStateFrame.can_dlc = 4;    // Data length: Assuming 4 byte is enough to command the state change
//    setStateFrame.data[0] = 0x08; // Example payload value for closed loop control, adjust as necessary
    uint32_t state = 8; // Assuming 8 is the value for AXIS_STATE_CLOSED_LOOP_CONTROL
    memcpy(setStateFrame.data, &state, sizeof(state)); // This ensures all 4 bytes are set correctly

    if (write(s, &setStateFrame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      perror("Write (Set State)");
      return 1;
    }

// Prepare the CAN frame
    struct can_frame velocityCommandFrame;
    velocityCommandFrame.can_id = (0x00 << 5) | 0x0D; // Combine your function code and node ID
    velocityCommandFrame.can_dlc = 8; // Data length code: 8 bytes (4 for velocity, 4 for torque feed-forward)

// Set input velocity to 20
    float inputVelocity = 20.0; // Your desired velocity
// Assuming your system uses IEEE 754 floats directly (common in C++ implementations),
// just copy the memory representation:
    memcpy(velocityCommandFrame.data, &inputVelocity, sizeof(inputVelocity));

// The next 4 bytes are for torque feed-forward, set to zero if not used
    float torqueFF = 0.0; // Not using torque feed-forward in this example
    memcpy(velocityCommandFrame.data + 4, &torqueFF, sizeof(torqueFF));

// Write the frame to the CAN socket
    if (write(s, &velocityCommandFrame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      perror("Write (Set Velocity)");
      return 1; // Return with error
    }


    sleep(2);

// Prepare the CAN frame
    velocityCommandFrame;
    velocityCommandFrame.can_id = (0x00 << 5) | 0x0D; // Combine your function code and node ID
    velocityCommandFrame.can_dlc = 8; // Data length code: 8 bytes (4 for velocity, 4 for torque feed-forward)

// Set input velocity to 20
    inputVelocity = 0.0; // Your desired velocity
// Assuming your system uses IEEE 754 floats directly (common in C++ implementations),
// just copy the memory representation:
    memcpy(velocityCommandFrame.data, &inputVelocity, sizeof(inputVelocity));

// The next 4 bytes are for torque feed-forward, set to zero if not used
    torqueFF = 0.0; // Not using torque feed-forward in this example
    memcpy(velocityCommandFrame.data + 4, &torqueFF, sizeof(torqueFF));

// Write the frame to the CAN socket
    if (write(s, &velocityCommandFrame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      perror("Write (Set Velocity)");
      return 1; // Return with error
    }

    sleep(1);

    setStateFrame.can_id = 0x00 << 5 | 0x07; // Example CAN ID for setting axis state, adjust as necessary
    setStateFrame.can_dlc = 4;    // Data length: Assuming 4 byte is enough to command the state change
//    setStateFrame.data[0] = 0x08; // Example payload value for closed loop control, adjust as necessary
    state = 1; // Assuming 8 is the value for AXIS_STATE_CLOSED_LOOP_CONTROL
    memcpy(setStateFrame.data, &state, sizeof(state)); // This ensures all 4 bytes are set correctly

    if (write(s, &setStateFrame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      perror("Write (Set State)");
      return 1;
    }


    // Continuous loop to read all received frames
    for(int i = 0; i < 10; i++) {
      int nbytes = read(s, &frame, sizeof(struct can_frame)); // Read a frame
      if (nbytes < 0) {
        perror("Read");
        break; // Exit the loop if there is a read error
      } else if (nbytes == sizeof(struct can_frame)) {
        std::cout << "Received frame with ID: " << std::hex << frame.can_id;
        std::cout << " and data: ";
        for (int k = 0; k < frame.can_dlc; k++) {
          std::cout << std::hex << static_cast<int>(frame.data[k]) << " ";
        }
        std::cout << std::endl;
      }
      // Add some exit condition if necessary, for example:
      // if (frame.can_id == some_specific_id) break;
    }


    std::cout << "Received frame with ID: " << std::hex << frame.can_id;
    std::cout << " and data: " << std::hex << (int) frame.data[0];
    std::cout << (int) frame.data[1] << std::endl;

    close(s); // Close the socket
    return 0;
  }

  ~ODriveCanInterface() {
    std::cout << "Destructor" << std::endl;
  }

};

#endif //HIVE_CORE_LIBRARIES_INCLUDE_ODRIVE_CAN_INTERFACE_ODRIVE_CAN_INTERFACE_H_
