cmake_minimum_required(VERSION 2.8.3)
project(davinci_pcl_utils)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   
cs_add_library(davinci_pcl_utils src/davinci_pcl_utils.cpp)  

# Executables
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)
cs_add_executable(davinci_pcl_utils_main src/davinci_pcl_utils_main.cpp)
cs_add_executable(davinci_pcl_utils_main2 src/davinci_pcl_utils_main2.cpp)

target_link_libraries(davinci_pcl_utils_main davinci_pcl_utils ${catkin_LIBRARIES})
target_link_libraries(davinci_pcl_utils_main2 davinci_pcl_utils ${catkin_LIBRARIES})

cs_install()
cs_export()
    
