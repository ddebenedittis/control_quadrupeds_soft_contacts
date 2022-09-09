# ==============================================================================
#                             PROJECT CONFIGURATION                             
# ==============================================================================

cmake_minimum_required(VERSION 3.5)
project(teleoperate_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



# ==============================================================================
#                               FIND DEPENDENCIES                               
# ==============================================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(generalized_pose_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)



# ==============================================================================
#                                ADD EXECUTABLES                                
# ==============================================================================

set(EXECUTABLE_NAME teleop_robot_base_node)

set(EXECUTABLE_DEPENDENCIES
    rclcpp

    gazebo_msgs
    generalized_pose_msgs
    std_msgs
    sensor_msgs
)

add_executable(${EXECUTABLE_NAME} src/teleop_robot_base_node.cpp)

target_include_directories(${EXECUTABLE_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${EXECUTABLE_NAME} PUBLIC ${EXECUTABLE_DEPENDENCIES})

ament_export_targets(${EXECUTABLE_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${EXECUTABLE_DEPENDENCIES})

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS ${EXECUTABLE_NAME}
    EXPORT ${EXECUTABLE_NAME}_targets
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
)



if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()