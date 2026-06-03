cmake_minimum_required(VERSION 3.12)
project(perfect_drone_sim)

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

# generate compile_commands.json for VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# Record ROS distro name (e.g., 'humble', 'galactic'...)
set(ROS_DISTRO_ENV "$ENV{ROS_DISTRO}")

# supported ROS2 version: galactic, humble and higher version
if(ROS_VERSION_ENV STREQUAL "2")
  if(ROS_DISTRO_ENV STREQUAL "galactic")
    message("\n${BoldGreen}ROS2 version: ${ROS_DISTRO_ENV} =galactic detected...${ColourReset}\n")
  else()
    message("\n${BoldGreen}ROS2 version: ${ROS_DISTRO_ENV} >galactic detected...${ColourReset}\n")
  endif()
endif()

if(ROS_VERSION_ENV STREQUAL "1")

message("\n${BoldGreen}Building for ROS1...${ColourReset}\n")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        mavros_msgs
        geometry_msgs
        nav_msgs
        message_generation
        mars_quadrotor_msgs
        visualization_msgs
        rosfmt
        message_filters
        marsim_render
        )

add_service_files(
        FILES
        SetUavPose.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
        mavros_msgs
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
#        -ldw
        )


add_executable(perfect_drone
        src/ros1_perfect_drone_node.cpp
        )
target_link_libraries(perfect_drone
        ${THIRD_PARTY}
        )

add_executable(so3_quadrotor_sim
  src/so3_quadrotor_sim.cpp
)
target_link_libraries(so3_quadrotor_sim
  ${THIRD_PARTY}
)
add_dependencies(so3_quadrotor_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  mars_quadrotor_msgs_generate_messages_cpp
)

endif()


# ROS2 CMakelist
if(ROS_VERSION_ENV STREQUAL "2")

message("\n${BoldGreen}Building for ROS2...${ColourReset}\n")

## ROS Related packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(marsim_render REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Other packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/SetUavPose.srv"
)


include_directories(
        SYSTEM
        include
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${marsim_render_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
        )

find_package(marsim_render REQUIRED)

set(dependencies
        marsim_render
        rclcpp
        sensor_msgs
        mavros_msgs
        mars_quadrotor_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        tf2_ros
)

add_executable(perfect_drone
        src/ros2_perfect_drone_node.cpp
)

ament_target_dependencies(perfect_drone PUBLIC
        ${dependencies}
)
target_link_libraries(perfect_drone PUBLIC
        ${THIRD_PARTY}
)

target_include_directories(perfect_drone PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_executable(so3_quadrotor_sim
        src/so3_quadrotor_sim.cpp
)
ament_target_dependencies(so3_quadrotor_sim PUBLIC
        ${dependencies}
)
target_link_libraries(so3_quadrotor_sim PUBLIC
        ${THIRD_PARTY}
)
# NOTE: use service or msg in same package, the CMake code is different in ROS2 galactic and humble,
#       so we reference the CMakeLists.txt of livox_ros_driver2.
if(ROS_DISTRO_ENV STREQUAL "galactic") # ROS2 galactic
set(PROJECT_INTERFACE_TARGET "${PROJECT_NAME}__rosidl_typesupport_cpp")
add_dependencies(so3_quadrotor_sim ${PROJECT_NAME})
get_target_property(PROJECT_INTERFACES_INCLUDE_DIRECTORIES ${PROJECT_INTERFACE_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
else() # ROS2 >= humble
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(so3_quadrotor_sim "${cpp_typesupport_target}")
endif()
target_include_directories(so3_quadrotor_sim PUBLIC
        ${PROJECT_INTERFACES_INCLUDE_DIRECTORIES}
)
target_link_libraries(so3_quadrotor_sim PUBLIC
        ${PROJECT_INTERFACE_TARGET}
)

# 注册 导出头文件
install(
        DIRECTORY include/
        DESTINATION include
)

# 注册 导出库文件
install(
        TARGETS
        perfect_drone
        so3_quadrotor_sim
        EXPORT
        perfect_drone
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
# install the launch file
install(DIRECTORY
        launch
        rviz
        rviz2
        meshes
        config
        DESTINATION share/${PROJECT_NAME}/
        )


ament_package()

endif()
