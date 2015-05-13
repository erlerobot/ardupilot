cmake_minimum_required(VERSION 2.8.3)
project(apm_rc)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
##add_message_files(DIRECTORY msg FILES apm_baro.msg)
add_message_files(DIRECTORY msg FILES)

 # Hardware Abstraction Layer APM ROS definitions
 #		TODO: create another package that abstracts
 #				all boards
add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_LINUX)
add_definitions(-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_LINUX_PXF)
##add_definitions(-DSKETCHNAME="APMRover2")

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#SET(CMAKE_CXX_FLAGS "fpermissive")
add_definitions ( -fpermissive)

add_executable(apm_rc_main src/apm_rc_main.cpp	)

target_link_libraries(apm_rc_main ${catkin_LIBRARIES})
add_dependencies(apm_rc_main apm_rc_generate_messages_cpp)

#file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/examples/*)
#  catkin_install_python(PROGRAMS
#  ${SCRIPTS}
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(
#	TARGETS apm_rc_main
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
