cmake_minimum_required(VERSION 3.8)
project(haptiquad_contacts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mujoco_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(haptiquad_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Open3D REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# ament_python_install_package(${PROJECT_NAME})


# install(
#   PROGRAMS  scripts/contact_publisher.py
#   DESTINATION lib/${PROJECT_NAME}
# )


add_executable(contact_estimator 
      src/contact_estimator.cpp
)

ament_target_dependencies(contact_estimator
  mujoco_msgs
  geometry_msgs
  Eigen3
  rclcpp
  visualization_msgs
  haptiquad_msgs
  tf2_ros
  tf2_geometry_msgs
)


install(
    TARGETS contact_estimator
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config assets scripts
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(contact_estimator 
  Open3D::Open3D
  ${Open3D_LIBRARIES}
)
link_directories(${Open3D_LIBRARY_DIRS})



include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
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
