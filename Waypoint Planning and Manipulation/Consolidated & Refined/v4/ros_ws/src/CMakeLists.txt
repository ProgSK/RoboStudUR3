cmake_minimum_required(VERSION 2.8.3)
project(genwbtest)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(genWBtest ./genWBtest.cpp)
target_link_libraries(genWBtest ${catkin_LIBRARIES})
