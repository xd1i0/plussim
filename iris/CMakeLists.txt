project(iris) # TODO adapt to visual project name

# Export compile commands for IDE support (Zed, VSCode, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# ============================================
# Fetch raylib
# ============================================
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

# -------------------------------------------------------------------------------------------
# ============================================
# Setup for using ImGui with raylib
# ============================================
# ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.4
)
FetchContent_MakeAvailable(imgui)

# rlImGui
FetchContent_Declare(
    rlimgui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG main
)
FetchContent_MakeAvailable(rlimgui)

# ============================================
# Build ImGui as a library
# ============================================
add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
)

# ============================================
# Build rlImGui as a library
# ============================================
add_library(rlimgui STATIC
    ${rlimgui_SOURCE_DIR}/rlImGui.cpp
)
target_include_directories(rlimgui PUBLIC 
  ${rlimgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}
)
target_link_libraries(rlimgui PUBLIC imgui raylib)
# -------------------------------------------------------------------------------------------

# add the .h and .cpp files from the src directory
file(GLOB_RECURSE HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# add the .h and .cpp files from the src directory
file(GLOB_RECURSE EXT_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/external/self/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/external/self/*.h
)

# add normal executable
add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SRC_FILES} ${EXT_FILES})

# link the executable to raylib
target_link_libraries(${PROJECT_NAME} PUBLIC rlimgui stellaris)

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