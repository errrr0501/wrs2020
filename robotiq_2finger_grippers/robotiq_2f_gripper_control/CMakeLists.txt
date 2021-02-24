cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_gripper_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
 
    roslib
    geometry_msgs
    robotis_device
    robotis_controller
    robotis_controller_msgs
    manipulator_h_base_module_msgs
    manipulator_h_base_module
    genmsg 
    actionlib_msgs 
    actionlib
    robotiq_2f_gripper_msgs
  COMPONENTS
    roscpp
    rospy
    std_msgs
    robotiq_2f_gripper_msgs

    
)


find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

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
   INCLUDE_DIRS include  
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS std_msgs robotiq_2f_gripper_msgs roscpp roslib std_msgs geometry_msgs actionlib 
  DEPENDS Boost EIGEN3  
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME} src/gripper_control.cpp)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robotiq_2f_gripper_control.cpp
# )


#############
## Install ##
#############

catkin_install_python(PROGRAMS 
      scripts/robotiq_2f_action_server.py
      scripts/robotiq_2f_action_client_example.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
add_library(${PROJECT_NAME} src/gripper_control.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(gripper src/gripper_control.cpp)
target_link_libraries(gripper ${catkin_LIBRARIES})
#add_dependencies(talker dual_arm_control_generate_messages_cpp)

