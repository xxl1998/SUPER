cmake_minimum_required(VERSION 3.5)
project(rog_map)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)

# reduce warning
add_compile_options(-Wno-dev)
cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0054 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress developer warnings")

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)

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
        visualization_msgs
        message_filters
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rog_map
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
        -ldw
        yaml-cpp
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp include/*.cpp)
add_library(rog_map STATIC ${srcs})

target_include_directories(rog_map PUBLIC include)
target_link_libraries(rog_map PUBLIC
        ${THIRD_PARTY})

endif()


# ROS2 CMakelist
if(ROS_VERSION_ENV STREQUAL "2")

message("\n${BoldGreen}Building for ROS2...${ColourReset}\n")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

set(includes
        include
        ${Eigen_INCLUDE_DIRS}
        #            ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(third_party_libs
        #            ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
)

set(ros_libs
        rclcpp
        std_msgs
        nav_msgs
        geometry_msgs
        visualization_msgs
        pcl_conversions
        tf2_ros
)

include_directories(
        ${includes}
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp include/*.cpp)

add_library(rog_map STATIC ${srcs})

target_include_directories(rog_map PUBLIC include)
target_link_libraries(rog_map PUBLIC
        ${THIRD_PARTY})


ament_target_dependencies(${PROJECT_NAME} PUBLIC
        ${ros_libs}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${third_party_libs}
)


install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME}
        ${third_party_libs}
)
ament_export_include_directories(${includes})
install(DIRECTORY include/
        DESTINATION include/
)

ament_package()

endif()
