project(cyber_py_wrapper)

#include cyberRT_project directories
include_directories(${cyberRT_project_BINARY_DIR})
include_directories(${cyberRT_project_SOURCE_DIR})

find_package(Python3 COMPONENTS Development)
include_directories(${Python3_INCLUDE_DIRS})

#build _cyber.so
add_library(_cyber_py3 SHARED "py_cyber.cc")
target_link_libraries(_cyber_py3 cyber)
install(CODE "file(RENAME ${cyberRT_project_BINARY_DIR}/lib_cyber_py3.so ${cyberRT_project_BINARY_DIR}/_cyber_py3.so)")
install(FILES "${cyberRT_project_BINARY_DIR}/_cyber_py3.so" DESTINATION ${CMAKE_INSTALL_LIBDIR}/cyber/cyber_py3)

#build _cyber_record.so
add_library(_cyber_record_py3 SHARED "py_record.cc")
target_link_libraries(_cyber_record_py3 cyber)
install(CODE "file(RENAME ${cyberRT_project_BINARY_DIR}/lib_cyber_record_py3.so ${cyberRT_project_BINARY_DIR}/_cyber_record_py3.so)")
install(FILES "${cyberRT_project_BINARY_DIR}/_cyber_record_py3.so" DESTINATION ${CMAKE_INSTALL_LIBDIR}/cyber/cyber_py3)

#build _cyber_time.so
add_library(_cyber_time_py3 SHARED "py_time.cc")
target_link_libraries(_cyber_time_py3 cyber)
install(CODE "file(RENAME ${cyberRT_project_BINARY_DIR}/lib_cyber_time_py3.so ${cyberRT_project_BINARY_DIR}/_cyber_time_py3.so)")
install(FILES "${cyberRT_project_BINARY_DIR}/_cyber_time_py3.so" DESTINATION ${CMAKE_INSTALL_LIBDIR}/cyber/cyber_py3)

#build _cyber_timer.so
add_library(_cyber_timer_py3 SHARED "py_timer.cc")
target_link_libraries(_cyber_timer_py3 cyber)
install(CODE "file(RENAME ${cyberRT_project_BINARY_DIR}/lib_cyber_timer_py3.so ${cyberRT_project_BINARY_DIR}/_cyber_timer_py3.so)")
install(FILES "${cyberRT_project_BINARY_DIR}/_cyber_timer_py3.so" DESTINATION ${CMAKE_INSTALL_LIBDIR}/cyber/cyber_py3)

#build _cyber_parameter_py3.so
add_library(_cyber_parameter_py3 SHARED "py_parameter.cc")
target_link_libraries(_cyber_parameter_py3 cyber)
install(CODE "file(RENAME ${cyberRT_project_BINARY_DIR}/lib_cyber_parameter_py3.so ${cyberRT_project_BINARY_DIR}/_cyber_parameter_py3.so)")
install(FILES "${cyberRT_project_BINARY_DIR}/_cyber_parameter_py3.so" DESTINATION ${CMAKE_INSTALL_LIBDIR}/cyber/cyber_py3)