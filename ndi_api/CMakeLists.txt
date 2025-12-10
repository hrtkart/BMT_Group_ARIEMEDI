# library/CMakeLists.txt
cmake_minimum_required(VERSION 3.28)
project(NDICAPI)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify the directories for source files and includes
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(INCLUDE_DIR_SRC ${SOURCE_DIR}/include)
set(SOURCE_FILES_EXAMPLE ${CMAKE_CURRENT_SOURCE_DIR}/example.cpp)

# Find all source files
file(GLOB LIB_SOURCES ${SOURCE_DIR}/*.cpp)

# Create the library
add_library(ndicapi SHARED ${LIB_SOURCES})

# Include directories
target_include_directories(ndicapi PUBLIC ${INCLUDE_DIR} ${INCLUDE_DIR_SRC})

add_executable(example ${SOURCE_FILES_EXAMPLE})
target_link_libraries(example ndicapi)