# ==============================================================================
#                             PROJECT CONFIGURATION                             
# ==============================================================================

cmake_minimum_required(VERSION 3.5)
project(whole_body_controller)

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

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

find_package(hierarchical_optimization REQUIRED)
find_package(robot_model REQUIRED)



# ==============================================================================
#                                 ADD LIBRARIES                                 
# ==============================================================================

set(LIBRARY_NAME ${PROJECT_NAME})

set(LIBRARY_DEPENDENCIES
    Eigen3
    pinocchio

    hierarchical_optimization
    robot_model
)

add_library(${LIBRARY_NAME} SHARED
    src/deformations_history_manager.cpp
    src/control_tasks.cpp
    src/mpc_control_tasks.cpp
    src/prioritized_tasks.cpp
    src/mpc_prioritized_tasks.cpp
    src/whole_body_controller.cpp
    src/mpc_whole_body_controller.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${LIBRARY_NAME} PUBLIC ${LIBRARY_DEPENDENCIES})

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
)



# ==============================================================================
#                                   ADD TESTS                                   
# ==============================================================================

add_executable(TestDeformationsHistoryManager test/test_deformations_history_manager.cpp)

target_include_directories(TestDeformationsHistoryManager PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(TestDeformationsHistoryManager PUBLIC ${LIBRARY_NAME})
ament_target_dependencies(TestDeformationsHistoryManager PUBLIC Eigen3)

# ==============================================================================

add_executable(TestControlTasks test/test_control_tasks.cpp)

target_include_directories(TestControlTasks PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(TestControlTasks PUBLIC ${LIBRARY_NAME})
ament_target_dependencies(TestControlTasks PUBLIC Eigen3)

# ==============================================================================

add_executable(TestPrioritizedTasks test/test_prioritized_tasks.cpp)

target_include_directories(TestPrioritizedTasks PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(TestPrioritizedTasks PUBLIC ${LIBRARY_NAME})
ament_target_dependencies(TestPrioritizedTasks PUBLIC Eigen3)

# ==============================================================================

add_executable(TestWBC test/test_whole_body_controller.cpp)

target_include_directories(TestWBC PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(TestWBC PUBLIC ${LIBRARY_NAME})
ament_target_dependencies(TestWBC PUBLIC Eigen3)



if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
