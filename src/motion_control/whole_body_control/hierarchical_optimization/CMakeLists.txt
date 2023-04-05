# ==============================================================================
#                             PROJECT CONFIGURATION                             
# ==============================================================================

cmake_minimum_required(VERSION 3.5)
project(hierarchical_optimization)

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
find_package(quadprog REQUIRED)



# ==============================================================================
#                                 ADD LIBRARIES                                 
# ==============================================================================

add_library(${PROJECT_NAME} SHARED src/hierarchical_qp.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} Eigen3 quadprog)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 quadprog)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)



# ==============================================================================
#                                   ADD TESTS                                   
# ==============================================================================

if(BUILD_TESTING)
    # find_package(ament_lint_auto REQUIRED)
    # ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_test test/test_hierarchical_optimization.cpp)

    target_include_directories(${PROJECT_NAME}_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
    )
    ament_target_dependencies(${PROJECT_NAME}_test Eigen3 quadprog)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()



ament_package()
