cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2_control)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_DEPENDS
    ament_cmake
    rclcpp
    hardware_interface
    pluginlib
    controller_manager
    joint_limits
    urdf
    glfw3
)


# Define directories, and create directories if they do not exist
set(MUJOCO_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
file(MAKE_DIRECTORY ${MUJOCO_DOWNLOAD_DIR})

# Define the base URL and version for the library
set(MUJOCO_VERSION 3.2.6)
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(MUJOCO_PLATFORM linux-aarch64)
else ()
    set(MUJOCO_PLATFORM linux-x86_64)
endif ()
include(FetchContent)
FetchContent_Declare(
    mujocoDownload
    URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-${MUJOCO_PLATFORM}.tar.gz
    UPDATE_COMMAND ""
    SOURCE_DIR ${MUJOCO_DOWNLOAD_DIR}
)
FetchContent_MakeAvailable(mujocoDownload)

find_library(MUJOCO_LIB mujoco HINTS ${MUJOCO_DOWNLOAD_DIR}/lib)

include_directories(include)

add_library(mujoco_system_plugins SHARED src/mujoco_system.cpp)
ament_target_dependencies(mujoco_system_plugins ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_system_plugins ${MUJOCO_LIB})
target_include_directories(mujoco_system_plugins PUBLIC ${MUJOCO_DOWNLOAD_DIR}/include ${EIGEN3_INCLUDE_DIR})

install(TARGETS
    mujoco_system_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# TODO: make it simple
add_executable(mujoco_ros2_control src/mujoco_ros2_control_node.cpp src/mujoco_rendering.cpp src/mujoco_ros2_control.cpp)
ament_target_dependencies(mujoco_ros2_control ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_ros2_control ${MUJOCO_LIB} glfw)
target_include_directories(mujoco_ros2_control PUBLIC ${MUJOCO_DOWNLOAD_DIR}/include ${EIGEN3_INCLUDE_DIR})

install(TARGETS
    mujoco_ros2_control
    DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.clang-format")
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_system_plugins.xml)
ament_package()
