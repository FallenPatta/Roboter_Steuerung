#cmake_minimum_required(VERSION 2.8)
#project( DisplayImage )
#find_package( OpenCV REQUIRED )
#add_executable( DisplayImage planar_tracking.cpp )
#target_link_libraries( DisplayImage ${OpenCV_LIBS})


cmake_minimum_required(VERSION 3.1)

project(image_tracking)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED roscpp roslib message_generation geometry_msgs cmake_modules OpenCV)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node src/planar_tracking.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
