cmake_minimum_required(VERSION 2.8.3)
project(iiwa_hw_grl)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#find_package(grl REQUIRED)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  tf
  urdf
  cmake_modules
  iiwa_msgs
  iiwa_ros
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
	CATKIN_DEPENDS  controller_interface
			hardware_interface
			control_toolbox
)

set(GRL_DIR ~/ws_gsk/grl)
set(FRI_SDK_DIR ~/ws_gsk/grl)
message(ERROR ${grl_INCLUDE_DIRS} ${grl_LIBRARIES})

set(grl_LIBS libKukaFRIClient.a libnanopb.a)

## Specify additional locations of header files
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${grl_INCLUDE_DIRS}
  #${GRL_DIR}/include
  ${FRI_SDK_DIR}/data/FRI-Client-SDK_Cpp/src/base
  ${FRI_SDK_DIR}/data/FRI-Client-SDK_Cpp/src/protobuf_gen
  ${FRI_SDK_DIR}/data/FRI-Client-SDK_Cpp/src/nanopb-0.2.8
  ${FRI_SDK_DIR}/data/FRI-Client-SDK_Cpp/src/protobuf
  ${FRI_SDK_DIR}/data/FRI-Client-SDK_Cpp/include
)

#include_directories(${GRL_DIR})
include_directories(${GRL_DIR}/include)
include_directories(${GRL_DIR}/build/include)

link_directories(${GRL_DIR}/build/lib)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/iiwa_hw.cpp
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}-bin
    src/main.cpp
)

## Add dependence to the iiwa_msg module for the executable
add_dependencies(${PROJECT_NAME}-bin
				iiwa_msgs_generate_messages_cpp)

## Add dependence to the iiwa_msg module for the library
add_dependencies(${PROJECT_NAME}
				iiwa_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-bin
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${grl_LIBS}
    ${Boost_LIBRARIES}
)

add_executable(grl_kuka_ros_driver
    src/grl_kuka_ros_driver.cpp
)
add_dependencies(grl_kuka_ros_driver
        iiwa_msgs_generate_messages_cpp)

target_link_libraries(grl_kuka_ros_driver
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${grl_LIBS}
    ${Boost_LIBRARIES}
)