cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp geometry_msgs cv_bridge sensor_msgs vision_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(vision_node
                src/node.cpp
                src/QrReader.cpp
                include/QrReader.h
                src/DistanceCalculator.cpp
                include/DistanceCalculator.h
                src/VisionNode.cpp
                include/VisionNode.h
                include/json.hpp)

target_link_libraries(vision_node zbar ${catkin_LIBRARIES})
set_property(TARGET vision_node PROPERTY CXX_STANDARD 11)

install(TARGETS vision_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#add_executable(${PROJECT_NAME}_node
#        src/node.cpp
#        src/QrReader.cpp
#        src/QrReader.h
#        src/DistanceCalculator.cpp
#        src/DistanceCalculator.h
#        include/zbar.h)