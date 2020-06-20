cmake_minimum_required(VERSION 2.8.3)
project(ldws_detector)

add_compile_options(-fopenmp)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS
   roscpp
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenMP REQUIRED)

add_library(${PROJECT_NAME}
  src/AMF.cpp
  src/Bspline.cpp
  src/Caractere.cpp
  src/Classement_Zone.cpp
  src/Detection_Zone.cpp
  src/fileUtil.cpp
  src/Init_Struct.cpp
  src/Initialisation.cpp
  src/Matrice.cpp
  src/Median.cpp
  src/Mise_A_Jour.cpp
  src/Points.cpp
  src/Recherche.cpp
  src/Road_Tracker.cpp
  src/Sauvegarde.cpp
  src/LDWS_AlarmDecision.cpp
  src/LDWS_Interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
