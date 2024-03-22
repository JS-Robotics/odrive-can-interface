//
// Created by JS-Robotics on 16.03.24.
//


#ifndef ODRIVECANINTERFACE_INCLUDE_CAN_INTERFACE_MANAGER_H_
#define ODRIVECANINTERFACE_INCLUDE_CAN_INTERFACE_MANAGER_H_

#include <string>
#include <unordered_map>
#include <thread>
#include <mutex>

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

#include <bitset>

#include "can_message_handler.h"

class CanInterfaceManager {
 public:
  // Delete copy constructor and copy assignment operator
  CanInterfaceManager(const CanInterfaceManager &) = delete;
  CanInterfaceManager &operator=(const CanInterfaceManager &) = delete;

  // Static method for accessing the single instance
  static CanInterfaceManager &GetInstance(const std::string &interface_name, uint32_t read_time_ms = 10) {
    static CanInterfaceManager instance(interface_name, read_time_ms);
    return instance;
  }

  void AddODriveInterface(int node_id, CanMessageHandler *interface) {
    odrive_interfaces_[node_id] = interface;
    std::cout << "Added device: " << node_id << std::endl;

  };

  bool StartCanLink() {
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW); // Create the socket
    if (socket_ < 0) {
      perror("Socket");
      return false;
    }

    int flags = fcntl(socket_, F_GETFL, 0);
    if (flags == -1) {
      perror("fcntl(F_GETFL)");
      close(socket_);
      return false;
    }
    flags |= O_NONBLOCK;
    if (fcntl(socket_, F_SETFL, flags) == -1) {
      perror("fcntl(F_SETFL)");
      close(socket_);
      return false;
    }

    sockaddr_can addr{};
    ifreq ifr{};
    std::cout << "Can Interface socket name set to: " << interface_name_ << std::endl;
    strcpy(ifr.ifr_name, interface_name_.c_str());
    ioctl(socket_, SIOCGIFINDEX, &ifr); // Specify the interface you wish to use

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind the socket to the network interface
    if (bind(socket_, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
      perror("Bind");
      close(socket_);
      return false;
    }

    std::cout << "Bound socket correctly starting reading thread" << std::endl;
    read_thread_ =
        std::thread(&CanInterfaceManager::ReadCanMessages, this); // Starting thread if socket bind is correct.
    return true;
  }

  void SendCanMessage(const can_frame &frame) const {
//    std::cout << "WritingCanMessage" << std::endl;
    if (write(socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      perror("Write");
    }
  }

 private:
  explicit CanInterfaceManager(const std::string &interface_name, uint32_t read_time_ms) :
      interface_name_(interface_name),
      socket_(-1),
      read_time_ms_(read_time_ms) {
    std::cout << "CanInterfaceManager Constructor with interface name: '" << interface_name_ << "'. With read time: "
              << read_time_ms << "[ms] - " << static_cast<uint32_t>(1000 / read_time_ms_) << "[Hz]" << std::endl;
  };

  ~CanInterfaceManager() {
    std::cout << "CanInterfaceManager Destructor with interface name: " << interface_name_ << std::endl;
    stop_thread_ = true;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    close(socket_); // Close the socket
  }

  void DispatchMessage(int node_id, const can_frame &frame) {
    std::lock_guard<std::mutex> guard(map_mutex_);  // Assuming multithreaded access
    auto it = odrive_interfaces_.find(node_id);
    if (it != odrive_interfaces_.end()) {
      it->second->HandleCanMessage(frame);  // Call the message handler for the corresponding ODrive interface.
    } else {
      std::cerr << "No ODriveCanInterface registered for node ID " << node_id << std::endl;
    }
  }

  void ReadCanMessages() {
    while (!stop_thread_) {
      can_frame frame{};
      if (ReadCanFrame(frame)) {
//        std::cout << "Read Frame" << std::endl;
        if (frame.can_id != 0x21 && frame.can_id
            != 0x29) {  // 0x21 and 0x29 Are not related to the axis, Seems like 0x21 is the heartbeat, not sure about the other
          int node_id = ExtractNodeId(frame); // Assuming you implement this
          DispatchMessage(node_id, frame);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(read_time_ms_));  //TODO() make system the ensure packages are emptied fast enough instead
    }
  }

  bool ReadCanFrame(can_frame &frame) {
    int64_t nbytes = read(socket_, &frame, sizeof(struct can_frame)); // Problem using socket_ here
    if (nbytes < 0) {
      return false;
    } else if (nbytes == sizeof(struct can_frame)) {
    }
    return true;
  }

  int ExtractNodeId(const can_frame &frame) {
    int can_id = static_cast<int>(frame.can_id);
    int node_id = (can_id & kIdMaskFilter_) >> 5;
    return node_id;

  }

  std::unordered_map<int, CanMessageHandler *> odrive_interfaces_;
  std::string interface_name_;
  bool stop_thread_ = false;
  std::thread read_thread_;
  std::mutex map_mutex_;
  int socket_;
  uint32_t read_time_ms_;  //Default to 10ms [100Hz]

  static constexpr int
      kIdMaskFilter_ = 0x7FFFF0;// Masks to preserve only relevant node_id bits and shifts (Zeros the 4 last bits)
};
#endif //ODRIVECANINTERFACE_INCLUDE_CAN_INTERFACE_MANAGER_H_
