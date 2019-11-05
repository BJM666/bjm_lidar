cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(lidar_pose)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3)

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES testmp.cpp)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

include_directories(${PCL_INCLUDE_DIRS}
/usr/include/
${EIGEN3_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS}
/usr/lib
${EIGEN_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
add_executable(lidar_pose lidar_pose.cpp)
target_link_libraries(lidar_pose ${PCL_LIBRARIES})
target_link_libraries(lidar_pose ${EIGEN_LIBRARIES})
