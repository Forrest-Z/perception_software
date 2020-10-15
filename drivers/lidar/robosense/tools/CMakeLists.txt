project(rs_driver_tools)

#include drivers directories
include_directories(${drivers_BINARY_DIR})
include_directories(${drivers_SOURCE_DIR})

add_executable(robosense_demo_online demo_online.cpp)
target_link_libraries(robosense_demo_online
    ${rs_driver_LIBRARIES}
    boost_system
    pcap 
    pthread     
)

add_executable(robosense_demo_pcap demo_pcap.cpp)
target_link_libraries(robosense_demo_pcap
    ${rs_driver_LIBRARIES}
    boost_system
    pcap 
    pthread
)

        