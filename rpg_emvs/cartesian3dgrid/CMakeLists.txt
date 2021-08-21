project(cartesian3dgrid)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)

# Adding PCL lib
find_package(PCL 1.8 REQUIRED)
# Setting include, lib directories and definitions 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS} )


find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories( ${Boost_INCLUDE_DIR} ${Eigen3_INCLUDE_DIRS} ) 

set(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "-O3 -std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB SOURCE_FILES src/*.cpp)

cs_add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME})

#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

cs_export()

