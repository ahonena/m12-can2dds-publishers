cmake_minimum_required(VERSION 3.10.2)
project (CANread_event_publishers_1)


add_executable(CANread_event_publishers_1 ${CMAKE_SOURCE_DIR}/src/CANread_event_publishers_1.cpp)

target_link_libraries( CANread_event_publishers_1
pcanbasic
)