cmake_minimum_required(VERSION 3.5)
project(marsim_render)

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)  # Options: Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "-O3 -Wall")
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS  "-O -Wall -g")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Add compiler flags to treat all warnings as errors
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Enable all common warnings and treat them as errors
    message(STATUS "GCC or Clang detected, adding -Wall -Wextra -Werror")
    add_compile_options(-Wall -Wextra -Werror -Wunused)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # Enable warning level 4 and treat all warnings as errors for MSVC
    message(STATUS "MSVC detected, adding /W4 /WX")
    add_compile_options(/W4 /WX)
endif()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wnon-virtual-dtor -Woverloaded-virtual")
endif ()

# FIXME! This is a temporary fix to suppress warnings from glad
add_compile_options(-Wno-pedantic)



# Add preprocessor definitions
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

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        include
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIRS}
        ${glfw3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp
        DEPENDS OpenCV
)


add_library(GLAD include/glad/glad.c)


add_library(${PROJECT_NAME}
        src/marsim_render.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${glfw3_LIBRARIES}
        ${PCL_LIBRARIES}
        OpenMP::OpenMP_CXX
        GLAD
        glfw
        dl
        yaml-cpp
)





add_executable(test_render
    test/main.cpp
)

target_link_libraries(test_render PUBLIC
        ${PROJECT_NAME}
)

endif()


# ROS2 CMakelist
if(ROS_VERSION_ENV STREQUAL "2")

message("\n${BoldGreen}Building for ROS2...${ColourReset}\n")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)


set(includes
        include
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIRS}
        ${glfw3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(third_party_libs
        ${OpenCV_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${glfw3_LIBRARIES}
        ${PCL_LIBRARIES}
        OpenMP::OpenMP_CXX
        glfw
        dl
        yaml-cpp
)

set(ros_libs
        rclcpp
        std_msgs
)


include_directories(
        ${includes}
)

add_library(${PROJECT_NAME} SHARED
        include/glad/glad.c
        src/marsim_render.cpp
)


ament_target_dependencies(${PROJECT_NAME} PUBLIC
        ${ros_libs}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${third_party_libs}
)

ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        OpenMP
)

install(TARGETS
        ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_include_directories(${includes})
install(DIRECTORY include/
        DESTINATION include/
)

ament_package()

endif()
