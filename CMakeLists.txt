cmake_minimum_required(VERSION 3.31)
project(plussim)

set(CMAKE_CXX_STANDARD 20)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose: Debug Release RelWithDebInfo MinSizeRel" FORCE)
endif()

# Export compile commands for IDE support (Zed, VSCode, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

file(GLOB_RECURSE HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)


add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC raylib)
