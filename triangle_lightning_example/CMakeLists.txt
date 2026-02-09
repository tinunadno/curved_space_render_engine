cmake_minimum_required(VERSION 3.15)

project(triangle_lightning_example LANGUAGES CXX)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(triangle_lightning_example
        main.cpp
)

target_link_libraries(triangle_lightning_example
        PRIVATE
        shader_core
        model_render_core
        glfw
        OpenGL::GL
)
