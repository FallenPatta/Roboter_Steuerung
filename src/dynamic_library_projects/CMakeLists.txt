cmake_minimum_required(VERSION 3.5)

project(dynamic_library_projects)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Debug)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp roslib tf message_generation sensor_msgs std_msgs cmake_modules pcl_conversions pcl_ros)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(unlinked_main src2/main.cpp)

add_executable(dll_main src1/dynamicLoad.cpp src1/VariablePrinterObject.cpp src1/VariablePrinterObjectFactory.cpp)

add_executable(pointCloud_node src3/point_cloud_reader_node.cpp
src3/SLAM/Mapper.cpp src3/SLAM/Localizer.cpp src3/SLAM/PoseEstimator.cpp
src3/SLAM/ICP.cpp src3/SLAM/KalmanFilter.cpp src3/SLAM/GraphMap.cpp)

add_library(printlibrary SHARED
    src1/MessagePrinter.cpp
    src1/HelloWorldPrinter.cpp
    src1/GoodbyeWorldPrinter.cpp
    src1/PrinterComposite.cpp
    src1/DeallocatingPrinterComposite.cpp
    src1/PluginProject.cpp
)

include_directories(printlibrary PRIVATE include1)
include_directories(printlibrary PRIVATE src1)

include_directories(unlinked_main PRIVATE include2)
include_directories(unlinked_main PRIVATE src2)

include_directories(pointCloud_node PRIVATE include3)
include_directories(pointCloud_node PRIVATE src3)

target_link_libraries(dll_main dl)

target_link_libraries(pointCloud_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
