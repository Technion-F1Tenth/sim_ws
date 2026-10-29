cmake_minimum_required(VERSION 3.8)
project(mrs_roboracer_ompl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ompl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(planning_node src/planning_node.cpp)

ament_target_dependencies(planning_node
        rclcpp
        std_msgs
        nav_msgs
        geometry_msgs
        ackermann_msgs
        std_srvs
        tf2
        tf2_geometry_msgs
)

# ---- Link Eigen and OMPL (PLAIN signature) ----
target_include_directories(planning_node SYSTEM PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(planning_node
        Eigen3::Eigen
        ${OMPL_LIBRARIES}
        yaml-cpp
)

# ---- Optional RCS project ----
if(EXISTS "/home/andy/CLionProjects/RCS/CMakeLists.txt")
  message(STATUS "Adding external RCS project from /home/andy/CLionProjects/RCS")
  add_subdirectory("/home/andy/CLionProjects/RCS" ${CMAKE_CURRENT_BINARY_DIR}/RCS_build EXCLUDE_FROM_ALL)
  # Link the planner library into planning_node (rcs is the library target created above)
  target_link_libraries(planning_node rcs)
  target_compile_definitions(planning_node PRIVATE HAS_RCS=1)
endif()

target_include_directories(planning_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(planning_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS planning_node
        DESTINATION lib/${PROJECT_NAME})

# Launch files live in mrs_roboracer_bringup (ompl_pure_pursuit_launch.py)
install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
