cmake_minimum_required(VERSION 2.8.3)
project(hercules_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager geometry_msgs hardware_interface
  roscpp roslaunch sensor_msgs std_msgs
  topic_tools hercules_msgs)
find_package(Boost REQUIRED COMPONENTS thread chrono)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(hercules_node
  src/hercules_base.cpp
  src/HerculesRobot.cpp)
target_link_libraries(hercules_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# Build the GPIO node
add_executable(gpio_node
        src/gpio_node.cpp)
target_link_libraries(gpio_node ${catkin_LIBRARIES})
