cmake_minimum_required(VERSION 2.8.3)
project(autonomous_landing)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs image_transport cv_bridge aruco)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(${PROJECT_NAME}_node src/main.cpp) 
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node_2 src/offb_node.cpp) 
target_link_libraries(${PROJECT_NAME}_node_2 ${catkin_LIBRARIES})

add_executable(camera_node src/camera_node.cpp) 
target_link_libraries(camera_node ${catkin_LIBRARIES})



