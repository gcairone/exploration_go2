cmake_minimum_required(VERSION 3.8)
project(explorer_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dipendenze ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)   
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

# --- Nodo map_projector ---
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(map_projector src/map_projector.cpp)
ament_target_dependencies(map_projector 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  pcl_conversions 
  pcl_ros
)
target_link_libraries(map_projector ${PCL_LIBRARIES})

add_executable(map_projector_bayesian src/map_projector_bayesian.cpp)
ament_target_dependencies(map_projector_bayesian 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  pcl_conversions 
  pcl_ros
)
target_link_libraries(map_projector_bayesian ${PCL_LIBRARIES})


install(TARGETS
  map_projector
  map_projector_bayesian
  DESTINATION lib/${PROJECT_NAME}
)

# --- Config e launch ---
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.launch.py"
)

# --- Test ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
