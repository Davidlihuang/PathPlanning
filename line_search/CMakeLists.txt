cmake_minimum_required(VERSION 3.20.0)
project(lst)
set(CMAKE_DISABLE_SOURCE_CHAGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
 
add_executable(lst mikami.c)
target_link_libraries(lst PUBLIC m)
target_include_directories(lst  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
