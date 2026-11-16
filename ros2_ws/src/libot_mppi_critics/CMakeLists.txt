cmake_minimum_required(VERSION 3.8)
project(libot_mppi_critics)

find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(xtensor REQUIRED)

# Jazzy binaries may lag the jazzy branch and omit these two upstream critics.
# Provide matching plugin implementations only when the installed MPPI package
# does not already export them.
get_filename_component(
  NAV2_MPPI_SHARE "${nav2_mppi_controller_DIR}/.." ABSOLUTE)
set(NAV2_MPPI_CRITICS_XML "${NAV2_MPPI_SHARE}/critics.xml")
file(READ "${NAV2_MPPI_CRITICS_XML}" NAV2_MPPI_CRITICS_MANIFEST)

string(FIND
  "${NAV2_MPPI_CRITICS_MANIFEST}"
  "mppi::critics::OmniConstraintCritic"
  OMNI_CRITIC_INDEX)
string(FIND
  "${NAV2_MPPI_CRITICS_MANIFEST}"
  "mppi::critics::MecanumCritic"
  MECANUM_CRITIC_INDEX)

set(COMPAT_SOURCES)
set(PLUGIN_DESCRIPTION_FILES)

if(OMNI_CRITIC_INDEX EQUAL -1)
  list(APPEND COMPAT_SOURCES src/omni_constraint_critic.cpp)
  list(APPEND PLUGIN_DESCRIPTION_FILES omni_constraint_critic.xml)
endif()

if(MECANUM_CRITIC_INDEX EQUAL -1)
  list(APPEND COMPAT_SOURCES src/mecanum_critic.cpp)
  list(APPEND PLUGIN_DESCRIPTION_FILES mecanum_critic.xml)
endif()

if(COMPAT_SOURCES)
  add_library(ot_mppi_critics SHARED ${COMPAT_SOURCES})
  target_compile_features(ot_mppi_critics PUBLIC cxx_std_17)
  target_include_directories(ot_mppi_critics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(ot_mppi_critics
    nav2_mppi_controller
    nav2_core
    nav2_costmap_2d
    pluginlib
    rclcpp
    rclcpp_lifecycle
    xtensor)

  install(TARGETS ot_mppi_critics
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  install(DIRECTORY include/ DESTINATION include)
  foreach(plugin_description_file ${PLUGIN_DESCRIPTION_FILES})
    pluginlib_export_plugin_description_file(
      nav2_mppi_controller ${plugin_description_file})
  endforeach()
endif()

ament_package()
