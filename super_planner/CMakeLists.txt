cmake_minimum_required(VERSION 3.5)
project(super_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=return-type)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=unused-but-set-variable)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer")

# generate compile_commands.json for VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DFMT_HEADER_ONLY)
# backward-cpp: enable DWARF-based symbol resolution (file:line)
add_definitions(-DBACKWARD_HAS_DW=1)
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
        rog_map
        yaml-cpp
        tf
)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex iostreams)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
catkin_package(
        INCLUDE_DIRS
        LIBRARIES super_planner
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
        ${Boost_LIBRARIES}
        #        super_planner
        yaml-cpp
        -ldw
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(super STATIC ${srcs})
target_include_directories(super PUBLIC
        include)
target_link_libraries(super PUBLIC
)


add_executable(fsm_node
        Apps/fsm_node_ros1.cpp
)
target_link_libraries(fsm_node
        super
        ${THIRD_PARTY}
)

add_executable(read_replan_log
        Apps/read_replan_log.cpp
)
target_link_libraries(read_replan_log
        super
        -lncurses
        ${THIRD_PARTY}
)


add_executable(traj_opt_tuning
        Apps/traj_opt_tuning.cpp
)
target_link_libraries(traj_opt_tuning
        super
        -lncurses
        ${THIRD_PARTY}
)

endif()


# ROS2 CMakelist
if(ROS_VERSION_ENV STREQUAL "2")

message("\n${BoldGreen}Building for ROS2...${ColourReset}\n")

include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rog_map REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
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
        -ldw
)

set(ros_libs
        rclcpp
        std_msgs
        mars_quadrotor_msgs
        sensor_msgs
        tf2_ros
        nav_msgs
        geometry_msgs
        visualization_msgs
        rog_map
        tf2_ros
        pcl_conversions

)

include_directories(
        ${includes}
)


#=================FOR libsuper ============================#
#    file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(super
        ${srcs}
)
target_include_directories(super PUBLIC
        ${includes})
ament_target_dependencies(super PUBLIC
        ${ros_libs}
)

#=================FOR Benchmark Node ============================#
add_executable(fsm_node
        Apps/fsm_node_ros2.cpp
)
target_link_libraries(fsm_node PUBLIC
        super
        ${third_party_libs}
)
ament_target_dependencies(fsm_node PUBLIC
        ${ros_libs}
)

#=================INSTALL all targets ============================#

install(
        TARGETS
        fsm_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(${includes})

install(DIRECTORY include/
        DESTINATION include/
)

# install the launch file
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)


ament_package()

endif()
