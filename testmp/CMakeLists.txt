cmake_minimum_required(VERSION 2.7)
project(openmptest)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES testmp.cpp)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(openmptest ${SOURCE_FILES})

target_link_libraries(openmptest ${OpenMP_LIBS})
