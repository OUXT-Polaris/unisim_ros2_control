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
ament_auto_add_library(unisim_ros2_control_component SHARED
  src/unisim_ros2_control.cpp
)
target_link_libraries(unisim_ros2_control_component pugixml ${Boost_LIBRARIES} crypto ssl cpprest)

rclcpp_components_register_nodes(unisim_ros2_control_component
  "unisim_ros2_control::UniSimRos2ControlComponent")

ament_auto_add_executable(unisim_ros2_control_node
  src/unisim_ros2_control.cpp
  src/unisim_ros2_control_node.cpp
)
target_link_libraries(unisim_ros2_control_node pugixml ${Boost_LIBRARIES} crypto ssl cpprest)

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
