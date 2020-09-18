project(ai_proto)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

file(GLOB AI_PROTO_SRCS *.proto)
PROTOBUF_GENERATE_CPP(	PROTO_SRCS PROTO_HDRS ${AI_PROTO_SRCS})

add_library(ai_proto SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(ai_proto ${Protobuf_LIBRARIES})
message(STATUS "protobuf_library: " ${Protobuf_LIBRARIES})

install(TARGETS ai_proto LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
