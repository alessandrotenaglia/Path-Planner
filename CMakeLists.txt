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
add_library(${PROJECT_NAME}_utils STATIC src/Drawer.cpp
                                         src/ExpMap.cpp
                                         src/Planner.cpp
                                         src/Point.cpp
                                         src/Util.cpp)
target_link_libraries(${PROJECT_NAME}_utils PUBLIC Boost::serialization)
target_link_libraries(${PROJECT_NAME}_utils PUBLIC ${EIGEN3_LIBS})
target_link_libraries(${PROJECT_NAME}_utils PUBLIC ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_utils PRIVATE ${Pangolin_LIBRARIES})

add_executable(gen_pntcloud test/gen_pntcloud.cpp)
target_link_libraries(gen_pntcloud PRIVATE ${PROJECT_NAME}_utils)

add_executable(view_nav_pntcloud test/view_nav_pntcloud.cpp)
target_link_libraries(view_nav_pntcloud PRIVATE ${PROJECT_NAME}_utils)

add_executable(view_exp_pntcloud test/view_exp_pntcloud.cpp)
target_link_libraries(view_exp_pntcloud PRIVATE ${PROJECT_NAME}_utils)

add_executable(gen_planner test/gen_planner.cpp)
target_link_libraries(gen_planner PRIVATE ${PROJECT_NAME}_utils)

add_executable(gen_explorer test/gen_explorer.cpp)
target_link_libraries(gen_explorer PRIVATE ${PROJECT_NAME}_utils)

add_executable(view_planner test/view_planner.cpp)
target_link_libraries(view_planner PRIVATE ${PROJECT_NAME}_utils)

add_executable(view_explorer test/view_explorer.cpp)
target_link_libraries(view_explorer PRIVATE ${PROJECT_NAME}_utils)

add_executable(planner test/planner.cpp)
target_link_libraries(planner PRIVATE ${PROJECT_NAME}_utils)

add_executable(explorer test/explorer.cpp)
target_link_libraries(explorer PRIVATE ${PROJECT_NAME}_utils)
