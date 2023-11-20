cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# # Set target C++ standard and required compiler features
# set(CMAKE_CXX_STANDARD 14 CACHE STRING "The target C++ standard. PCL requires C++14 or higher.")
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)
# set(PCL_CXX_COMPILE_FEATURES cxx_std_14)

project(PclTest)
 
find_package(PCL 1.13.1 REQUIRED)

#set(PCL_LIBRARY_DIRS "/usr/local/lib")
 
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# add_executable(Test2 Test2.cpp)
# target_link_libraries (Test2 ${PCL_LIBRARIES})

add_executable(Test1 Test1.cpp)
target_link_libraries (Test1 ${PCL_LIBRARIES})

# add_executable(SmoothTrangle SmoothTrangle.cpp)
# target_link_libraries (SmoothTrangle ${PCL_LIBRARIES})

# add_executable(FastTrangle FastTrangle.cpp)
# target_link_libraries (FastTrangle ${PCL_LIBRARIES})

# add_executable(ResamplePly ResamplePly.cpp)
# target_link_libraries (ResamplePly ${PCL_LIBRARIES})

# add_executable(Resample Resample.cpp)
# target_link_libraries (Resample ${PCL_LIBRARIES})

# message(STATUS "PCL_DIR = ${PCL_DIR}")
# message(STATUS "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
# message(STATUS "PCL_DEFINITIONS = ${PCL_DEFINITIONS}")
# message(STATUS "PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS}")
# message(STATUS "PCL_LIBRARIES = ${PCL_LIBRARIES}")

# set(PCL_LIBRARY_DIR "/usr/local/lib")
# set(PCL_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
# message(STATUS "PCL_LIBRARY_DIR = ${PCL_LIBRARY_DIR}")

# find_package(PCL 1.11.0 EXACT REQUIRED) 
# add_definitions( ${PCL_DEFINITIONS} )

# #-------修改为---：
# file(GLOB PCL_LIBRARIES "/usr/local/lib/libpcl_*")
# set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.13")

# INCLUDE_DIRECTORIES(${PROJECT_NAME} ${PCL_INCLUDE_DIRS})
# TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES})


#add_executable(pcl_test pcl_test.cpp)
#target_link_libraries (pcl_test ${PCL_LIBRARIES})

#add_executable (pcl_pcd pcl_pcd.cpp)
#target_link_libraries (pcl_pcd ${PCL_LIBRARIES})

#add_executable(cloudViewerTest cloudViewerTest.cpp)
#target_link_libraries (cloudViewerTest ${PCL_LIBRARIES})
#install(TARGETS cloudViewerTest RUNTIME DESTINATION bin)


add_executable(visualizor visualizor.cpp)
target_link_libraries (visualizor ${PCL_LIBRARIES})
# install(TARGETS visualizor RUNTIME DESTINATION bin)

#add_executable(3Transform 3Transform.cpp)
#target_link_libraries (3Transform ${PCL_LIBRARIES})
#install(TARGETS 3Transform RUNTIME DESTINATION bin)