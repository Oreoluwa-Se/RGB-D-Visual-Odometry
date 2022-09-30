cmake_minimum_required(VERSION 3.5)
project(my_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp -pthread")
# general paths
set(SLAM_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SLAM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SLAM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SLAM_INCLUDE_MY_SLAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/my_slam)
set(SLAM_BINARY_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)


# find dependencies - ros components
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL  REQUIRED COMPONENTS common io)
find_package(image_transport REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find dependencies - non ros related
find_package(OpenCV 4.5 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# eigen included
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# fmt
find_package(fmt REQUIRED)

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
    fmt::fmt
    Eigen3::Eigen
    g2o_core g2o_stuff 
    g2o_types_sba 
    g2o_solver_csparse 
    g2o_csparse_extension    
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${CSPARSE_LIBRARY}
	${PCL_LIBRARY_DIRS}
    pthread)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# paths to header files
set(PRIVATE_HEADERS
    ${SLAM_INCLUDE_DIR}/common.hpp
    ${SLAM_INCLUDE_DIR}/helper_modules.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/utils/image_funcs.hpp    
    ${SLAM_INCLUDE_MY_SLAM_DIR}/utils/poission_filter.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/utils/triangulate.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/dataset/config.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/dataset/reader.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/structures/map_point.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/structures/feature.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/structures/frame.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/structures/map.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/sensor/camera.hpp
    # ${SLAM_INCLUDE_MY_SLAM_DIR}/control/vio.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/control/frontend.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/control/g2o_types.hpp
    ${SLAM_INCLUDE_MY_SLAM_DIR}/control/backend.hpp
    # ${SLAM_INCLUDE_MY_SLAM_DIR}/control/viewer.hpp
) 

# paths to source files
set(PRIVATE_SOURCES
    ${SLAM_SOURCE_DIR}/utils/image_funcs.cpp
    ${SLAM_SOURCE_DIR}/dataset/config.cpp
    ${SLAM_SOURCE_DIR}/structures/map_point.cpp
    ${SLAM_SOURCE_DIR}/structures/frame.cpp
    ${SLAM_SOURCE_DIR}/structures/map.cpp
    ${SLAM_SOURCE_DIR}/sensor/camera.cpp
    ${SLAM_SOURCE_DIR}/control/backend.cpp
    # ${SLAM_SOURCE_DIR}/control/viewer.cpp
)

# add library
add_library(my_slam SHARED
            ${PRIVATE_HEADERS}
            ${PRIVATE_SOURCES})

# including libraries
target_include_directories(my_slam
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
        $<BUILD_INTERFACE:${SLAM_INCLUDE_DIR}>
        ${SLAM_BINARY_INCLUDE_DIR}
    PRIVATE
        ${SLAM_SOURCE_DIR})

# project to libraries        
target_link_libraries(my_slam ${THIRD_PARTY_LIBS})  

## adding component directory
add_subdirectory(src)	

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
