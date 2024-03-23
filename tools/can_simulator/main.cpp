//
// Created by sondre on 23.03.24.
//

#include <csignal>
#include <thread>

#include "can_simulator.h"


bool shutdown_requested = false;

inline void stop_handler(int) {
  shutdown_requested = true;
  std::cout << "preparing to shut down..." << std::endl;
}

inline void setup_signal_handlers() {
  signal(SIGINT, stop_handler);
  signal(SIGTERM, stop_handler);
}

int main(){
  setup_signal_handlers();

  CanSimulator can_simulator;

  std::cout << "Hello Example" << std::endl;

  while (!shutdown_requested) {
    can_simulator.SendToBus();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}