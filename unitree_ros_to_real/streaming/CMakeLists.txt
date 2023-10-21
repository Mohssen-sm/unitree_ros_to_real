cmake_minimum_required(VERSION 2.8.3)
project(streaming)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        )

catkin_package()

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
    src/StateEstimation.cpp
)

add_executable(streaming_data src/exe/node.cpp)
target_link_libraries(streaming_data ${EXTRA_LIBS} ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(streaming_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})