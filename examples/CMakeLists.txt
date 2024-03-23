
#include_directories(include)
add_executable(BaseExample base_examples.cpp)
target_link_libraries(BaseExample ODriveCanInterface)

add_executable(VelocityControlExample velocity_control.cpp)
target_link_libraries(VelocityControlExample ODriveCanInterface)

add_executable(ReadWrite development/can_read_write.cpp)
target_link_libraries(ReadWrite ODriveCanInterface)

add_executable(Read development/can_read.cpp)
target_link_libraries(Read ODriveCanInterface)

add_executable(CanId development/id_extractor.cpp)

