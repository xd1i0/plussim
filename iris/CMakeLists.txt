project(iris) # TODO adapt to visual project name

include(FetchContent)

# Fetch raylib
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

# add the .h and .cpp files from the src directory
file(GLOB_RECURSE HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# add the .h and .cpp files from the src directory
file(GLOB_RECURSE EXT_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/external/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/external/*.h
)

# add normal executable
add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SRC_FILES} ${EXT_FILES})

# link the executable to raylib
target_link_libraries(${PROJECT_NAME} PUBLIC raylib)


# -------------- tests -------------- # 

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  GIT_SHALLOW TRUE
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

file(GLOB_RECURSE TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)

set(PROJECT_TESTS "${PROJECT_NAME}_test")

add_executable(
  ${PROJECT_TESTS}
  ${TEST_FILES}
)
target_link_libraries(
  ${PROJECT_TESTS}
  GTest::gtest_main
  stellaris
)

include(GoogleTest)
gtest_discover_tests(${PROJECT_TESTS})