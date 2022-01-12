# build as top level project for testing, and developement
project(signalProcessing)
cmake_minimum_required(VERSION 3.10)
list(APPEND CMAKE_PREFIX_PATH $CMAKE_CURRENT_SOURCE_DIR/cmake)
message(

        set(CMAKE_PREFIX_PATH ))
add_subdirectory(test)
add_subdirectory(src)