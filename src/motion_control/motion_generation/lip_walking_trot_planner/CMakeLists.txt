# =========================== Project Configuration ============================

cmake_minimum_required(VERSION 3.8)
project(lip_walking_trot_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================= Find Dependencies ==============================

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(generalized_pose_msgs REQUIRED)
find_package(rviz_legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(velocity_command_msgs REQUIRED)

find_package(quadprog REQUIRED)

# =============================== Add Libraries ================================

set(LIBRARY_NAME ${PROJECT_NAME})

set(LIBRARY_DEPENDENCIES
    Eigen3

    controller_interface
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    pluginlib

    gazebo_msgs
    generalized_pose_msgs
    nav_msgs
    rviz_legged_msgs
    sensor_msgs
    velocity_command_msgs

    quadprog
)

add_library(${LIBRARY_NAME} SHARED
    src/quaternion_math.cpp
    src/interpolator.cpp
    src/lip_planner.cpp
    src/lip_walking_trot_planner_controller.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${LIBRARY_NAME} PUBLIC ${LIBRARY_DEPENDENCIES})

# Export hardware plugins
pluginlib_export_plugin_description_file(controller_interface lip_walking_trot_planner_controller.xml)

ament_export_targets(${LIBRARY_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${LIBRARY_DEPENDENCIES})

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
    TARGETS ${LIBRARY_NAME}
    EXPORT ${LIBRARY_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# ================================= Add Tests ==================================

if(BUILD_TESTING)
    # find_package(ament_lint_auto REQUIRED)
    # ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_test test/test_interpolator.cpp)

    target_include_directories(${PROJECT_NAME}_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
    )
    ament_target_dependencies(${PROJECT_NAME}_test ${LIBRARY_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

# ==============================================================================

ament_package()
