project(drivers_proto)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})
message(STATUS "drivers_SOURCE_DIR: " ${drivers_BINARY_DIR})
message(STATUS "drivers_BINARY_DIR: " ${drivers_SOURCE_DIR})

file(GLOB CYBER_PROTO_SRCS *.proto)
PROTOBUF_GENERATE_CPP(	PROTO_SRCS PROTO_HDRS ${CYBER_PROTO_SRCS})

add_library(drivers_proto SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(drivers_proto ${Protobuf_LIBRARIES})
message(STATUS "protobuf_library: " ${Protobuf_LIBRARIES})

install(TARGETS drivers_proto LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
