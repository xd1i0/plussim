cmake_minimum_required(VERSION 4.0)
project(plussim)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.5
)
FetchContent_MakeAvailable(raylib)

add_executable(${PROJECT_NAME} src/main.cpp
        src/double_pendulum.cpp
        src/double_pendulum.h)

target_link_libraries(${PROJECT_NAME} raylib)
