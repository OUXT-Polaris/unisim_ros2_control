cmake_minimum_required(VERSION 3.8)
project(unisim_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(cmake_commands)
set(cmake_configure_args -DCPPREST_ROOT=/usr/include/cpprest)
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
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/unisim_openapi_interface-prefix/src/unisim_openapi_interface
  DESTINATION
    include/unisim_client
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
