cmake_minimum_required(VERSION 3.15)
project(geometric_map_buffer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wunused-variable)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(geometric_map_buffer_node_parameters
  src/geometric_map_buffer_node_parameters.yaml
)

ament_auto_add_library(geometric_map_buffer SHARED
  src/geometric_map_buffer.cpp
)
ament_auto_add_library(geometric_map_buffer_components SHARED
  src/geometric_map_buffer_node.cpp
)
target_link_libraries(geometric_map_buffer
  Eigen3::Eigen
)
target_link_libraries(geometric_map_buffer_components
  geometric_map_buffer
  geometric_map_buffer_node_parameters
)
rclcpp_components_register_node(geometric_map_buffer_components
  PLUGIN "geometric_map_buffer::GeometricMapBufferNode"
  EXECUTABLE geometric_map_buffer_node
)
install(
  TARGETS
    geometric_map_buffer
    geometric_map_buffer_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
