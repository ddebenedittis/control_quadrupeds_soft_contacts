# ==============================================================================
#                             PROJECT CONFIGURATION                             
# ==============================================================================

cmake_minimum_required(VERSION 3.5)
project(temp_pkg) # TODO

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
find_package(Eigen3 REQUIRED)
find_package(eiquadprog REQUIRED)

add_executable(Test test/test.cpp)

target_include_directories(Test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(Test PUBLIC Eigen3 eiquadprog)



if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
