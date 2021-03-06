cmake_minimum_required(VERSION 3.5)
project(unisim_ros2_control)

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

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(pugixml REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)

ament_auto_find_build_dependencies()

set(cmake_commands)
set(cmake_configure_args -DCPPREST_ROOT=/usr/include/cpprest -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
include(ExternalProject)
externalproject_add(unisim_openapi_interface
  GIT_REPOSITORY https://github.com/OUXT-Polaris/unisim_openapi_interface.git
  GIT_TAG cpp_client
  TIMEOUT 6000
  ${cmake_commands}
  CMAKE_ARGS
    ${cmake_configure_args}
  STEP_TARGETS build
  INSTALL_COMMAND ""
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/unisim_openapi_interface-prefix/src/unisim_openapi_interface/lib
  DESTINATION
    lib/unisim_ros2_control
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/unisim_openapi_interface-prefix/src/unisim_openapi_interface
  DESTINATION
    include/unisim_ros2_control
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/unisim_openapi_interface-prefix/src/unisim_openapi_interface)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/unisim_openapi_interface-prefix/src/unisim_openapi_interface/model)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/unisim_openapi_interface-prefix/src/unisim_openapi_interface/lib)

ament_auto_add_library(unisim_ros2_control_component SHARED
  src/unisim_ros2_control.cpp
  src/unisim_client.cpp
)
set_property(TARGET unisim_ros2_control_component PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(unisim_ros2_control_component pugixml ${Boost_LIBRARIES} ssl crypto cpprest CppRestSwaggerClient)
add_dependencies(unisim_ros2_control_component unisim_openapi_interface)

rclcpp_components_register_nodes(unisim_ros2_control_component
  "unisim_ros2_control::UniSimRos2ControlComponent")

ament_auto_add_executable(unisim_ros2_control_node
  src/unisim_ros2_control.cpp
  src/unisim_ros2_control_node.cpp
)
set_property(TARGET unisim_ros2_control_node PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(unisim_ros2_control_node pugixml ${Boost_LIBRARIES} ssl crypto cpprest CppRestSwaggerClient)
add_dependencies(unisim_ros2_control_node unisim_openapi_interface)

install(TARGETS
  unisim_ros2_control_node
  DESTINATION lib/unisim_ros2_control
)
install(TARGETS
  unisim_ros2_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(unisim_ros2_control_component)

#############
## Install ##
#############
install(DIRECTORY launch xacro
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
