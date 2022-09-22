# ==============================================================================
#                             PROJECT CONFIGURATION                             
# ==============================================================================

cmake_minimum_required(VERSION 3.5)
project(hqp_controller)

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

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(generalized_pose_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(whole_body_controller REQUIRED)



# ==============================================================================
#                                 ADD LIBRARIES                                 
# ==============================================================================

set(PROJECT_DEPENDENCIES
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle

    gazebo_msgs
    generalized_pose_msgs
    geometry_msgs
    sensor_msgs

    whole_body_controller
)

add_library(${PROJECT_NAME} SHARED src/hqp_controller.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${PROJECT_DEPENDENCIES})

pluginlib_export_plugin_description_file(controller_interface hqp_controller.xml)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)



# ==============================================================================
#                                ADD EXECUTABLES                                
# ==============================================================================

set(NODE_DEPENDENCIES
    rclcpp

    gazebo_msgs
    generalized_pose_msgs
    sensor_msgs
    std_msgs

    whole_body_controller
)

set(NODE_NAME ${PROJECT_NAME}_node)

add_executable(${NODE_NAME} src/hqp_controller_node.cpp)

target_include_directories(${NODE_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${NODE_NAME} PUBLIC ${NODE_DEPENDENCIES})

# ament_export_targets(${NODE_NAME}_targets HAS_LIBRARY_TARGET)
# ament_export_dependencies(${NODE_DEPENDENCIES})

install(
    TARGETS ${NODE_NAME}
    DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
