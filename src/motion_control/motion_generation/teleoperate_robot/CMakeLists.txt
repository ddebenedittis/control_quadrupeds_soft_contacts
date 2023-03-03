# ==============================================================================
#                             PROJECT CONFIGURATION                             
# ==============================================================================

cmake_minimum_required(VERSION 3.5)
project(teleoperate_robot)

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



# ==============================================================================
#                               FIND DEPENDENCIES                               
# ==============================================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(generalized_pose_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(velocity_command_msgs REQUIRED)



# ==============================================================================
#                                ADD EXECUTABLES                                
# ==============================================================================

set(FILES teleop_robot_base_node teleop_velocity_command_node)

foreach(FILE IN LISTS FILES)
    message(${FILE})

    set(SOURCE_FILE ${FILE})

    message(${SOURCE_FILE})

    set(EXECUTABLE_DEPENDENCIES
        rclcpp

        gazebo_msgs
        generalized_pose_msgs
        std_msgs
        sensor_msgs
        velocity_command_msgs
    )

    add_executable(${SOURCE_FILE} src/${SOURCE_FILE}.cpp)

    target_include_directories(${SOURCE_FILE} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    )

    ament_target_dependencies(${SOURCE_FILE} PUBLIC ${EXECUTABLE_DEPENDENCIES})

    ament_export_targets(${SOURCE_FILE}_targets HAS_LIBRARY_TARGET)
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
        TARGETS ${SOURCE_FILE}
        EXPORT ${SOURCE_FILE}_targets
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include/${PROJECT_NAME}
    )
endforeach()



if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()