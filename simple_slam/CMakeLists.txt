cmake_minimum_required(VERSION 3.0.2)
project(simple_slam)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES simple_slam
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 각 실행 파일마다 고유한 타겟 이름을 사용하여 add_executable() 명령을 추가
add_executable(${PROJECT_NAME}_node src/simple_slam_node.cpp)
add_executable(${PROJECT_NAME}_node_class src/simple_slam_node_class.cpp)
add_executable(${PROJECT_NAME}_node_gaussian src/simple_slam_node_gaussian.cpp)
add_executable(${PROJECT_NAME}_node_median src/simple_slam_node_median.cpp)

# 각 실행 파일 타겟에 대해 target_link_libraries() 명령을 추가
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node_class
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node_gaussian
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node_median
  ${catkin_LIBRARIES}
)

