cmake_minimum_required(VERSION 3.5)
project(mike_av_stack_localization)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14 -O3)
# add_compile_definitions(CLI11_HAS_FILESYSTEM=0)

set(CXX_FLAGS "-pthread -fPIC -DNDEBUG -Wextra")
# set(CMAKE_CXX_FLAGS "${CXX_FLAGS} -Wl,-Bstatic -lboost_filesystem -Wl,-Bdynamic")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Setting all dependancies to one variable
set(${PROJECT_NAME}_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_geometry_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters visualization)

find_package(Eigen3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(include)

add_executable(${PROJECT_NAME}_node 
  src/localization.cpp
  src/helper.cpp
  src/scan_matching.cpp
  src/ndt.cpp
  src/icp.cpp
  src/icps.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node   
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_geometry_msgs)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME localization PREFIX "")
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} pcl_visualization)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${PROJECT_NAME}_DEPS)
#  INCLUDE_DIRS include
#  DEPENDS Eigen

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
