cmake_minimum_required(VERSION 3.8)
project(mrs_roboracer_pp_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Adding Eigen dependency
find_package(eigen3_cmake_module REQUIRED)  # provides a robust FindEigen3
find_package(Eigen3 REQUIRED)               # imports Eigen3::Eigen
find_package(ackermann_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node rclcpp std_msgs nav_msgs geometry_msgs Eigen3 ackermann_msgs std_srvs)


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

install(TARGETS
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
