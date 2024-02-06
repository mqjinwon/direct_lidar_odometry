cmake_minimum_required(VERSION 3.8)
project(direct_lidar_odometry)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED) # pcl_conversion

find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# Include directories
include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# make srv files
rosidl_generate_interfaces(${PROJECT_NAME}
    srv/SavePCD.srv
    srv/SaveTraj.srv
    DEPENDENCIES   
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Libraries
add_library(nanoflann STATIC src/nano_gicp/nanoflann.cc)
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(nanoflann ${PCL_LIBRARIES} ${cpp_typesupport_target})
ament_target_dependencies(nanoflann rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs pcl_ros)

add_library(nano_gicp STATIC src/nano_gicp/lsq_registration.cc src/nano_gicp/nano_gicp.cc)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} nanoflann)
if(OPENMP_FOUND)
  target_link_libraries(nano_gicp OpenMP::OpenMP_CXX)
endif()
target_include_directories(nano_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Executables
# add_executable(dlo_odom_node src/dlo/odom_node.cc src/dlo/odom.cc)
# ament_target_dependencies(dlo_odom_node rclcpp std_msgs sensor_msgs geometry_msgs)
# target_link_libraries(dlo_odom_node nano_gicp ${PCL_LIBRARIES} ${cpp_typesupport_target})
# if(OPENMP_FOUND)
#   target_link_libraries(dlo_odom_node OpenMP::OpenMP_CXX)
# endif()

add_executable(dlo_map_node src/dlo/map_node.cc src/dlo/map.cc)
ament_target_dependencies(dlo_map_node rclcpp std_msgs sensor_msgs geometry_msgs)
target_link_libraries(dlo_map_node nano_gicp ${PCL_LIBRARIES} ${cpp_typesupport_target})
if(OPENMP_FOUND)
  target_link_libraries(dlo_map_node OpenMP::OpenMP_CXX )
endif()

ament_package()

# Install targets
install(TARGETS
  nanoflann
  nano_gicp
  # dlo_odom_node
  dlo_map_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files, etc.
install(DIRECTORY
  launch
  cfg
  DESTINATION share/${PROJECT_NAME}/
)
