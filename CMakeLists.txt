cmake_minimum_required(VERSION 3.10.2)

project (CANread_event_publishers_1)

find_package(RTIConnextDDS REQUIRED
HINTS ${CMAKE_SOURCE_DIR})



set(DDS_TYPE_OBJECT_DIR
${CMAKE_SOURCE_DIR}/src/IDL/M12Typelib/objs
)


file(GLOB dds_typeobjs
${DDS_TYPE_OBJECT_DIR}/*.o
)


message(
"The DDS_TYPE_OBJECT_DIR is: ${DDS_TYPE_OBJECT_DIR}"
)
message(
"The object files are: ${dds_typeobjs}"
)





add_library(M12_CAN2DDS STATIC
  ${CMAKE_SOURCE_DIR}/src/M12_CAN2DDS.cpp
  ${dds_typeobjs}
)
target_compile_definitions(M12_CAN2DDS PUBLIC
-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT
)

set_property(TARGET M12_CAN2DDS PROPERTY CXX_STANDARD 11)

target_include_directories( M12_CAN2DDS PRIVATE
${CMAKE_SOURCE_DIR}/src
${CMAKE_SOURCE_DIR}/src/IDL/M12Typelib/include
${CONNEXTDDS_INCLUDE_DIRS}
)

target_link_libraries( M12_CAN2DDS PUBLIC
pcanbasic
${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
pthread
dl
)




add_executable(CANread_event_publishers_1
${CMAKE_SOURCE_DIR}/src/CANread_event_publishers_1.cpp
)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

target_include_directories( CANread_event_publishers_1 PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/IDL/M12Typelib/include
  ${CONNEXTDDS_INCLUDE_DIRS}
)

message(
  "CONNEXTDDS_CPP2_API_LIBRARIES: ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}"
)
target_link_libraries( CANread_event_publishers_1 PRIVATE
M12_CAN2DDS
)
