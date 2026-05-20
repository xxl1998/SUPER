cmake_minimum_required(VERSION 3.5)
project(mission_planner)

if(NOT CMAKE_C_STANDARD)
        set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
endif()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif ()
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")


if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

# ROS compatibility
set(ROS_VERSION_ENV "$ENV{ROS_VERSION}")
if(ROS_VERSION_ENV STREQUAL "1")
  if(DEFINED ENV{AMENT_PREFIX_PATH} AND NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
    message(FATAL_ERROR "Conflicting ROS environment detected: ROS_VERSION=1 but AMENT_PREFIX_PATH is set.")
  endif()
  message("\n${BoldGreen}ROS1 environment detected...${ColourReset}\n")
  add_compile_definitions(USE_ROS1)
elseif(ROS_VERSION_ENV STREQUAL "2")
  if(DEFINED ENV{CATKIN_ENV_HOOK_WORKSPACE} AND NOT "$ENV{CATKIN_ENV_HOOK_WORKSPACE}" STREQUAL "")
    message(FATAL_ERROR "Conflicting ROS environment detected: ROS_VERSION=2 but CATKIN_ENV_HOOK_WORKSPACE is set.")
  endif()
  message("\n${BoldGreen}ROS2 environment detected...${ColourReset}\n")
  add_compile_definitions(USE_ROS2)
else()
  message(FATAL_ERROR "Invalid or missing ROS_VERSION environment variable. Please source a ROS1 or ROS2 environment before configuring.")
endif()

if(ROS_VERSION_ENV STREQUAL "1")

message("\n${BoldGreen}Building for ROS1...${ColourReset}\n")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        mars_quadrotor_msgs
        visualization_msgs
        message_filters
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
        -ldw
        )

add_executable(waypoint_mission
        Apps/ros1_waypoint_mission.cpp
        )
target_link_libraries(waypoint_mission
        ${THIRD_PARTY}
        )

endif()


# ROS2 CMakelist
if(ROS_VERSION_ENV STREQUAL "2")

message("\n${BoldGreen}Building for ROS2...${ColourReset}\n")

## ROS Related packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Other packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


include_directories(
        SYSTEM
        include
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
        -ldw
        )

set(dependencies
        rclcpp
        mavros_msgs
        sensor_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
)

add_executable(waypoint_mission
        Apps/ros2_waypoint_mission.cpp
)

ament_target_dependencies(waypoint_mission PUBLIC
        ${dependencies}
)
target_link_libraries(waypoint_mission PUBLIC
        ${THIRD_PARTY}
)

target_include_directories(waypoint_mission PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 注册 导出头文件
install(
        DIRECTORY include/
        DESTINATION include
)

# 注册 导出库文件
install(
        TARGETS waypoint_mission # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT waypoint_mission
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# install the launch file
install(DIRECTORY
        launch
        config
        data
        DESTINATION share/${PROJECT_NAME}/
        )


ament_package()

endif()
