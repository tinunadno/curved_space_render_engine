cmake_minimum_required(VERSION 3.15)

project(shader_core_examples LANGUAGES CXX)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(shader_core_examples
#        color_check.cpp
        simple_ray_tracer.cpp
#        hole_frame_color_check.cpp
)

target_link_libraries(shader_core_examples
        PRIVATE
        shader_core
        glfw
        OpenGL::GL
)
