cmake_minimum_required(VERSION 3.31)
project(plussim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose: Debug Release RelWithDebInfo MinSizeRel" FORCE)
endif()

# Export compile commands for IDE support (Zed, VSCode, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add subdirectories
add_subdirectory(stellaris)
add_subdirectory(visual)
