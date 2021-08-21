project(mapper_emvs)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


# Adding PCL lib
find_package(PCL 1.8 REQUIRED)
# Setting include, lib directories and definitions 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS} )

#add vtk
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(${vtk_INCLUDE_DIRS})
link_directories(${vtk_LIBRARY_DIRS} )
add_definitions(${vtk_DEFINITIONS} )

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
# Setting include, lib directories and definitions 
include_directories( ${Boost_INCLUDE_DIR} ${Eigen3_INCLUDE_DIRS} ) 

set(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "-O3 -fopenmp -std=c++11 ${CMAKE_CXX_FLAGS}")

## Generate dynamic reconfigure parameters in the 'cfg' folder
##generate_dynamic_reconfigure_options(
##  cfg/EMVSCfg.cfg
##)

set(HEADERS
  include/mapper_emvs/mapper_emvs.hpp
  include/mapper_emvs/data_loading.hpp
  include/mapper_emvs/depth_vector.hpp
  include/mapper_emvs/trajectory.hpp
  include/mapper_emvs/geometry_utils.hpp
  include/mapper_emvs/median_filtering.hpp
  include/mapper_emvs/pc_geometry.hpp
)

set(SOURCES
  src/mapper_emvs.cpp
  src/data_loading.cpp
  src/median_filtering.cpp
  src/pc_geometry.cpp
)

option(DEFINE_USE_INVERSE_DEPTH "Use linear spacing in inverse depth (if OFF, will use linear spacing in depth)" OFF)
if(DEFINE_USE_INVERSE_DEPTH)
    add_definitions(-DUSE_INVERSE_DEPTH)
endif(DEFINE_USE_INVERSE_DEPTH)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Executables
################################################################################

# cs_add_executable(run_emvs src/main.cpp)
# target_link_libraries(run_emvs ${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

cs_add_executable(realtime_emvs src/realtime.cpp)
add_dependencies(realtime_emvs ${PROJECT_NAME}_gencfg)
target_link_libraries(realtime_emvs ${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

cs_add_executable (pcd_write src/White_Box_pcdGen.cpp)
target_link_libraries (pcd_write ${PROJECT_NAME} ${PCL_LIBRARIES})

##cs_add_executable (old_pcd_write src/old_plate_pcd.cpp)
##target_link_libraries (old_pcd_write ${PROJECT_NAME} ${PCL_LIBRARIES})

#cs_add_executable (pcl_testing src/realsensePCLwrapper.cpp)
#target_link_libraries (pcl_testing ${PROJECT_NAME} ${PCL_LIBRARIES} librealsense2.so ${OpenCV_LIBS})

#cs_add_executable (Analysis src/Analyze_results.cpp)
#target_link_libraries (Analysis ${PROJECT_NAME} ${PCL_LIBRARIES})

#cs_add_executable (icp_test src/test_icp.cpp)
#target_link_libraries (pcd_write ${PROJECT_NAME} ${PCL_LIBRARIES})
################################################################################




cs_install()
cs_export()

