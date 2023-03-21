# ==============================================================================
#                             PROJECT CONFIGURATION                             
# ==============================================================================

cmake_minimum_required(VERSION 3.5)
project(rviz_legged_plugins)

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

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(rviz_legged_msgs REQUIRED)

find_package(ignition-math6 REQUIRED)

find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rviz_default_plugins REQUIRED)



# ==============================================================================
#                              INSTALL DIRECTORIES                              
# ==============================================================================

set(LIBRARY_NAME ${PROJECT_NAME})

set(rviz_legged_plugins_headers_to_moc
    include/rviz_legged_plugins/displays/friction_cones_display.hpp
    include/rviz_legged_plugins/displays/external_wrench_display.hpp
    include/rviz_legged_plugins/displays/paths_display.hpp
)

foreach(header "${rviz_legged_plugins_headers_to_moc}")
  qt5_wrap_cpp(rviz_legged_plugins_moc_files "${header}")
endforeach()

set(rviz_legged_plugins_source_files
    src/displays/friction_cones_display.cpp
    src/displays/external_wrench_display.cpp
    src/displays/paths_display.cpp
)


add_library(${LIBRARY_NAME} SHARED
    ${rviz_legged_plugins_moc_files}
    ${rviz_legged_plugins_source_files}
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME} PUBLIC
    rviz_ogre_vendor::OgreMain
    rviz_ogre_vendor::OgreOverlay
)

target_link_libraries(${LIBRARY_NAME} PRIVATE
    ignition-math6
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${LIBRARY_NAME} PRIVATE ${LIBRARY_NAME}_BUILDING_LIBRARY)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(${LIBRARY_NAME}
    PUBLIC
    rviz_legged_msgs
    image_transport
    interactive_markers
    laser_geometry
    map_msgs
    nav_msgs
    rclcpp
    resource_retriever
    rviz_common
    rviz_rendering
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    urdf
    visualization_msgs
    rviz_default_plugins
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${LIBRARY_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
    rviz_legged_msgs
    image_transport
    interactive_markers
    laser_geometry
    map_msgs
    nav_msgs
    rclcpp
    resource_retriever
    rviz_common
    rviz_ogre_vendor
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    urdf
    visualization_msgs
    rviz_default_plugins
)

install(
    TARGETS ${LIBRARY_NAME}
    EXPORT ${LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
    DESTINATION "share/${PROJECT_NAME}"
)



# ==============================================================================
#                            INSTALL PYTHON MODULES                             
# ==============================================================================

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/ground_to_base_frame_broadcaster_node.py
    scripts/terrain_projector_node.py
    DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package(
    CONFIG_EXTRAS "rviz_legged_plugins-extras.cmake"
)
