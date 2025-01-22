cmake_minimum_required(VERSION 3.0.2)
project(livelybot_oled)


message(${CMAKE_HOST_SYSTEM_PROCESSOR})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  serial  
  livelybot_msg  
  livelybot_power
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES livelybot_oled
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  # DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(OLED_NODE_FILE
    src/sensor_actuator_status.cpp
    src/ip_addr.cpp
    src/oled_interface.cpp
)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

add_library(livelybot_oled STATIC ${OLED_NODE_FILE})
target_link_libraries(livelybot_oled ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

link_directories(lib)

add_executable(livelybot_oled_node src/oled_run.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(livelybot_oled_node livelybot_oled ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(livelybot_oled_node livelybot_oled ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()

add_executable(livelybot_oled_hd_test src/oled_hardware_test.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(livelybot_oled_hd_test livelybot_oled ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(livelybot_oled_hd_test livelybot_oled ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_livelybot_oled.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
