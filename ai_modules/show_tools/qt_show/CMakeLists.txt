project(qt_show)

#include ai_algorithm directories
include_directories(${ai_algorithm_BINARY_DIR})
include_directories(${ai_algorithm_SOURCE_DIR})

set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
message(STATUS "${OPENGL_INCLUDE_DIR}, ${GLEW_INCLUDE_DIR}")
message(STATUS "${OPENGL_LIBRARIES} , ${GLEW_LIBRARIES}")
include_directories(${OPENGL_INCLUDE_DIR})

set(Qt5_DIR /opt/Qt5.12.0/5.12.0/gcc_64/lib/cmake/Qt5)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui OpenGL)
message(STATUS "Found Qt5: ${Qt5_VERSION}")
#message(STATUS "Qt5Core: ${Qt5Core_INCLUDE_DIRS}, ${Qt5Core_LIBRARIES}")
#include_directories(${Qt5Core_INCLUDE_DIRS})
#message(STATUS "Qt5Widgets: ${Qt5Widgets_INCLUDE_DIRS}, ${Qt5Widgets_LIBRARIES}")
#include_directories(${Qt5Widgets_INCLUDE_DIRS})
#message(STATUS "Qt5Gui: ${Qt5Gui_INCLUDE_DIRS}, ${Qt5Gui_LIBRARIES}")
#include_directories(${Qt5Gui_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR})
message(STATUS "qt_show: ${PROJECT_SOURCE_DIR}")

#collect source files
file(GLOB QT_SRCS "${PROJECT_SOURCE_DIR}/*.cc")
file(GLOB QT_HDRS "${PROJECT_SOURCE_DIR}/*.h")
file(GLOB QT_UIS "${PROJECT_SOURCE_DIR}/*.ui")
file(GLOB QT_QRSCS "${PROJECT_SOURCE_DIR}/*.qrc")

qt5_wrap_cpp(MOC ${QT_HDRS})
qt5_wrap_ui(UI_HDRS ${QT_UIS})
qt5_add_resources(RCC ${QT_QRSCS})

add_definitions(
    ${Qt5Core_DEFINITIONS}
    ${Qt5Widgets_DEFINITIONS}
    ${Qt5Gui_DEFINITIONS}
)

add_executable(${PROJECT_NAME} ${QT_SRCS} ${QT_HDRS} ${MOC} ${UI_HDRS} ${RCC})
target_link_libraries(${PROJECT_NAME}
    ai_proto
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    cyber
    cyber_proto
    fastrtps
    glog
    gflags
    pthread
    GL
)