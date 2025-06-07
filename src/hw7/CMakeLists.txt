cmake_minimum_required(VERSION 3.8)
project(hw7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Tell the compiler where to find the include files.
include_directories(include)

# C++ packages need rclcpp.  Add in the other packages we need here, too.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

# We need this to make the PCL stuff work.
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add in the executables.
add_executable(background_remover src/background_remover.cpp)
target_link_libraries(background_remover ${PCL_LIBRARIES})
ament_target_dependencies(background_remover rclcpp sensor_msgs pcl_ros pcl_conversions)

add_executable(table_remover src/table_remover.cpp)
target_link_libraries(table_remover ${PCL_LIBRARIES})
ament_target_dependencies(table_remover rclcpp sensor_msgs pcl_ros pcl_conversions tf2 visualization_msgs)

# Make sure we install the nodes so that ros2 run can find them.
install(TARGETS
  background_remover
  table_remover

  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files so that ros2 launch can find them.
#install(DIRECTORY
#  launch
#
#  DESTINATION share/${PROJECT_NAME}
#)

# Install the include files so that other pacakges can find them.
install(DIRECTORY
  include

  DESTINATION include/${PROJECT_NAME}
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
