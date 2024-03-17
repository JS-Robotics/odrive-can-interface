//
// Created by sondre on 17.03.24.
//


#include <iostream>
#include <linux/can.h>
#include <bitset>

int main() {
  can_frame frame;

  // for 0x09
//  Mask bits: 00000000000111111111111111000000
//  Original CAN ID bits: 00000000000000000000000000001001
//  Original CAN ID bits: 00000000000000000000000001101001
//  Masked CAN ID bits: 00000000000000000000000000000000

//  frame.can_id = 0x21; 0x29
  frame.can_id = 0x1D;
  int cmd_id = static_cast<int>(frame.can_id);
  int node_id = 84;  // Expected node_id
  int can_id = (node_id << 5) | cmd_id;

  std::cout << "Constructed Command CAN ID: " << can_id << std::endl;

  int mask = 0x1FFFFF0;  // Odd filter
//  int mask = 0x1FFFFC0;  // Even filter
  std::bitset<32> maskBits(mask);
  std::cout << "Mask bits: " << maskBits << std::endl;


  std::bitset<32> originalBits(can_id);
  std::cout << "Original CAN ID bits: " << originalBits << std::endl;

  int maskedCanId = can_id & mask;
  std::bitset<32> maskedBits(maskedCanId);
  std::cout << "Masked CAN ID bits: " << maskedBits << std::endl;

  int extracted_node_id = (can_id & mask) >> 5;
  std::cout << "Extracted Node ID: " << extracted_node_id << std::endl;

  return 0;
}