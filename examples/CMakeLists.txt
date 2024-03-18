
#include_directories(include)
add_executable(BaseExample base_examples.cpp)
target_link_libraries(BaseExample ODriveCanInterface)

add_executable(VelocityControlExample velocity_control.cpp)
target_link_libraries(VelocityControlExample ODriveCanInterface)

add_executable(ReadWrite can_read_write.cpp)
target_link_libraries(ReadWrite ODriveCanInterface)

add_executable(Read can_read.cpp)
target_link_libraries(Read ODriveCanInterface)

add_executable(CanId id_extractor.cpp)

