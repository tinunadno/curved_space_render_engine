cmake_minimum_required(VERSION 3.15)

project(model_render_core_example LANGUAGES CXX)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(model_render_core_example
        main.cpp
)

target_link_libraries(model_render_core_example
        PRIVATE
        shader_core
        model_render_core
        glfw
        OpenGL::GL
)
