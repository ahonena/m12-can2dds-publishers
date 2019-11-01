cmake_minimum_required(VERSION 3.10.2)

project (CANread_event_publishers_1)

find_package(RTIConnextDDS REQUIRED
HINTS ${CMAKE_SOURCE_DIR})



set(DDS_TYPE_OBJECT_DIR
${CMAKE_SOURCE_DIR}/src/IDL/M12Typelib/objs
)

set(USED_M12_TYPES
WorkHydraulicsRequests
)

file(GLOB dds_typeobjs
${DDS_TYPE_OBJECT_DIR}/${USED_M12_TYPES}*.o
)


#

message(
"The DDS_TYPE_OBJECT_DIR is: ${DDS_TYPE_OBJECT_DIR}"
)
message(
"The object files are: ${dds_typeobjs}"
)






add_executable(CANread_event_publishers_1
${CMAKE_SOURCE_DIR}/src/CANread_event_publishers_1.cpp
${dds_typeobjs}
)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
target_compile_definitions(CANread_event_publishers_1 PUBLIC
-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT
)

target_include_directories( CANread_event_publishers_1 PRIVATE
${CMAKE_SOURCE_DIR}/src
${CMAKE_SOURCE_DIR}/src/IDL/M12Typelib/include
${CONNEXTDDS_INCLUDE_DIRS}
)

message(
  "CONNEXTDDS_CPP_API_LIBRARIES: ${CONNEXTDDS_CPP_API_LIBRARIES_RELEASE_STATIC}"
)
target_link_libraries( CANread_event_publishers_1 PUBLIC
pcanbasic
${CONNEXTDDS_CPP_API_LIBRARIES_RELEASE_STATIC}
pthread
dl

)
