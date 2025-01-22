cmake_minimum_required(VERSION 3.0.2)
project(livelybot_serial)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    livelybot_msg  
    serial
    dynamic_reconfigure
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(serial_SRCS
    src/lively_serial.cc
    src/hardware/motor.cc
    src/hardware/canport.cc
    src/hardware/canboard.cc
    src/hardware/robot.cc
)


generate_dynamic_reconfigure_options(
  cfg/robot_dynamic_config_20.cfg
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread
    CATKIN_DEPENDS roscpp std_msgs livelybot_msg serial dynamic_reconfigure
)
find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(livelybot_serial SHARED ${serial_SRCS})
add_dependencies(livelybot_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(livelybot_serial ${catkin_LIBRARIES} rt pthread ${SERIALPORT_LIBRARY})

link_directories(lib)

# # test
add_executable(ik_run test/test_ik_run.cpp)
target_link_libraries(ik_run livelybot_serial ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_motor test/test_motor.cpp)
target_link_libraries(test_motor livelybot_serial ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_reset_zero test/test_reset_zero.cpp)
target_link_libraries(test_reset_zero livelybot_serial ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_feedback test/test_feedback.cpp)
target_link_libraries(test_feedback livelybot_serial ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_motor_feedback test/test_motor_feedback.cpp)
target_link_libraries(test_motor_feedback livelybot_serial ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_motor_run test/test_motor_run.cpp)
target_link_libraries(test_motor_run livelybot_serial ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_jump test/test_jump.cpp)
target_link_libraries(test_jump livelybot_serial ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_reconfigure test/test_reconfigure.cpp)
target_link_libraries(test_reconfigure livelybot_serial ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_timeout test/test_timeout.cpp)
target_link_libraries(test_timeout livelybot_serial ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

