cmake_minimum_required(VERSION 3.8)
project(wall-e)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)
# Install rviz config files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)
# Install meshes
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)
# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)
# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
