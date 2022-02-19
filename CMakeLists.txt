cmake_minimum_required(VERSION 3.16)

project(Path-Planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies.
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

# Include directories of library dependencies.
include_directories(include/)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})

# Create a shared library with common source files.
add_library(${PROJECT_NAME}_utils STATIC src/AStar.cpp
                                         src/LPAStar.cpp
                                         src/Box.cpp
                                         src/Drawer.cpp
                                         src/Map.cpp
                                         src/Point.cpp
                                         src/Util.cpp)
target_link_libraries(${PROJECT_NAME}_utils PUBLIC Boost::serialization)
target_link_libraries(${PROJECT_NAME}_utils PUBLIC ${EIGEN3_LIBS})
target_link_libraries(${PROJECT_NAME}_utils PUBLIC ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_utils PRIVATE ${Pangolin_LIBRARIES})

add_executable(gen_pntcloud test/generate_pntcloud.cpp)
target_link_libraries(gen_pntcloud PRIVATE ${PROJECT_NAME}_utils)

add_executable(view_pntcloud test/view_pntcloud.cpp)
target_link_libraries(view_pntcloud PRIVATE ${PROJECT_NAME}_utils)

add_executable(gen_map test/generate_map.cpp)
target_link_libraries(gen_map PRIVATE ${PROJECT_NAME}_utils)

add_executable(view_map test/view_map.cpp)
target_link_libraries(view_map PRIVATE ${PROJECT_NAME}_utils)

add_executable(fibonacci test/fibonacci_heap.cpp)
target_link_libraries(fibonacci PRIVATE ${PROJECT_NAME}_utils)

add_executable(astar test/astar_map.cpp)
target_link_libraries(astar PRIVATE ${PROJECT_NAME}_utils)

add_executable(lpa test/lpa_map.cpp)
target_link_libraries(lpa PRIVATE ${PROJECT_NAME}_utils)
