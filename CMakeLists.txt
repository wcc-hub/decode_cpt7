cmake_minimum_required(VERSION 3.4.1)
project(Openpos)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/ReadCPT.cpp
)

add_executable(ReadCPTData ReadCPTData.cpp)
target_link_libraries(ReadCPTData ${PROJECT_NAME})