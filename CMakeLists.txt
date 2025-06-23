cmake_minimum_required(VERSION 3.8)
project(chassis_keyboard_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED libevdev)

add_compile_options(-Wall -Wextra -pedantic)

include_directories(
  include
  ${LIBEVDEV_INCLUDE_DIRS}
)

add_executable(teleop_node src/teleop.cpp src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp geometry_msgs nav_msgs)
target_link_libraries(teleop_node ${LIBEVDEV_LIBRARIES})

install(TARGETS
  teleop_node
  DESTINATION lib/${PROJECT_NAME}
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
