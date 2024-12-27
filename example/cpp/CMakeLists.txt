cmake_minimum_required(VERSION 3.16)
project(stand_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(stand_go2 stand_go2.cpp)
target_link_libraries(stand_go2  unitree_sdk2)

add_executable(stand_h1 stand_h1.cpp)
target_link_libraries(stand_h1  unitree_sdk2)

add_executable(h1_hand h1_hand.cpp)
target_link_libraries(h1_hand  unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)