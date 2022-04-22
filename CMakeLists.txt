cmake_minimum_required(VERSION 3.0.2)
project(swarm_sim)

find_package(catkin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow")

find_package(catkin REQUIRED COMPONENTS
    decentralized_path_auction
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    decentralized_path_auction
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/map_gen.cpp
    src/bin_router.cpp
    src/path_planner.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    tests/test.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()
