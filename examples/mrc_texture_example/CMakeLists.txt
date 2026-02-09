cmake_minimum_required(VERSION 3.15)

project(texture_example LANGUAGES CXX)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(texture_example
        main.cpp
)

target_compile_definitions(texture_example PRIVATE PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(texture_example
        PRIVATE
        engine_core
        model_render_core
        glfw
        OpenGL::GL
)
