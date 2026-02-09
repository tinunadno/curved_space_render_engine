cmake_minimum_required(VERSION 3.15)

project(engine_core_examples LANGUAGES CXX)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(shader_core_color_checker
        color_check.cpp
)

add_executable(shader_core_ray_tracer
        simple_ray_tracer.cpp
)

add_executable(shader_core_hole_frame_color_checker
        hole_frame_color_check.cpp
)

target_link_libraries(shader_core_color_checker
        PRIVATE
        engine_core
        glfw
        OpenGL::GL
)

target_link_libraries(shader_core_ray_tracer
        PRIVATE
        engine_core
        glfw
        OpenGL::GL
)

target_link_libraries(shader_core_hole_frame_color_checker
        PRIVATE
        engine_core
        glfw
        OpenGL::GL
)
