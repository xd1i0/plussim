cmake_minimum_required(VERSION 3.31)
project(stellaris)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose: Debug Release RelWithDebInfo MinSizeRel" FORCE)
endif()

# Export compile commands for IDE support (Zed, VSCode, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Create the library
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

# Make headers accessible
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Only add tests if this is built as the main project
# (i.e., when you build mylib separately)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    enable_testing()
    add_subdirectory(tests)
endif()