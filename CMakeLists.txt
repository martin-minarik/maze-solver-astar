cmake_minimum_required(VERSION 3.22)
project(maze)

set(CMAKE_CXX_STANDARD 11)

file(COPY maze.txt DESTINATION ${CMAKE_BINARY_DIR})

add_executable(maze main.cpp maze.cpp)
