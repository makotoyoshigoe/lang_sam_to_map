cmake_minimum_required(VERSION 3.8)
project(lang_sam_to_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(ros2_lang_sam_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(lang_sam_to_map
    src/lang_sam_to_map_main.cpp
    src/lang_sam_to_map.cpp
    src/rgb_image.cpp
)

ament_target_dependencies(lang_sam_to_map
    ament_cmake
    rclcpp
    sensor_msgs
    cv_bridge
    pcl_ros
    image_geometry
    message_filters
    ros2_lang_sam_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
	nav_msgs
)

target_include_directories(lang_sam_to_map PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	${OpenCV_INCLUDE_DIRS}
)

install(TARGETS lang_sam_to_map
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
   	launch
	rviz
    param
	DESTINATION share/${PROJECT_NAME}
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
