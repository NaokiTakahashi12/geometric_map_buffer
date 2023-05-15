cmake_minimum_required(VERSION 3.8)
project(geometric_map_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wunused-variable)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(geometric_map_server_node_parameters
  src/geometric_map_server_node_parameters.yaml
)

ament_auto_add_library(geometric_map_server_component SHARED
  src/geometric_map_server_node.cpp
)
ament_target_dependencies(geometric_map_server_component
  geometric_map_buffer
)
target_include_directories(geometric_map_server_component
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(geometric_map_server_component
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  geometric_map_server_node_parameters
)
rclcpp_components_register_node(geometric_map_server_component
  PLUGIN "geometric_map_server::GeometricMapServerNode"
  EXECUTABLE geometric_map_server_node
)
install(
  TARGETS
    geometric_map_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY
    maps
    config
    launch
  DESTINATION
    "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
