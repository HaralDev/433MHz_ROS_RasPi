cmake_minimum_required(VERSION 2.8.3)
project(rf_communication)

## Compile as C++0x, supported in ROS Kinetic and newer, and needed for the rc-switch library
add_compile_options(-std=c++0x)
## Rc-switch library needs this definition to know we are using a Raspberry. The dependence on this definition was removed, as it would sometimes raise the error: fatal error: WProgram.h: No such file or directory.
add_definitions(-DRPI)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################




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
  LIBRARIES rc-switch spdlog get_desktop
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS message_runtime
)

# Python support install, should be called after catkin_package because only then is {CATKIN_PACKAGE_BIN_DESTINATION} made
catkin_install_python(PROGRAMS src/send_node src/desktop_path src/comms_data_eval.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# RC SWITCH library
add_library(rcswitch include/rc-switch/RCSwitch.cpp)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rf_communication.cpp
# )


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


# RFSNIFFER executable
add_executable(RFSniffer src/RFSniffer.cpp)
target_link_libraries(RFSniffer ${catkin_LIBRARIES} rcswitch -lwiringPi -lwiringPiDev -lcrypt)


# CODESEND executable
add_executable(codesend src/codesend.cpp)
target_link_libraries(codesend ${catkin_LIBRARIES} rcswitch -lwiringPi -lwiringPiDev -lcrypt)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rf_communication.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
