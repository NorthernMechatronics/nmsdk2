cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.13.0)


set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmsis-nn.cmake)

project(CMSIS-NN)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../../cmsis/CMSIS-NN ${PROJECT_BINARY_DIR}/cmsis/CMSIS-NN)

install (
    TARGETS cmsis-nn
    DESTINATION ${PROJECT_SOURCE_DIR}/lib
)