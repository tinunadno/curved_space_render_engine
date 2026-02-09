cmake_minimum_required(VERSION 3.15)

project(black_hole_shader LANGUAGES CXX)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(black_hole_shader
        main.cpp
)

target_link_libraries(black_hole_shader
        PRIVATE
        shader_core
        ray_marching_core
        glfw
        OpenGL::GL
)
