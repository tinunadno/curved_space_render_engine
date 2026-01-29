cmake_minimum_required(VERSION 3.15)

project(curved_space_render_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

add_compile_definitions(ENABLE_LOG)

add_subdirectory(shader_core)
add_subdirectory(shader_core_examples)
add_subdirectory(ray_marching_core)
add_subdirectory(black_hole_shader)
add_subdirectory(model_render_core)
add_subdirectory(model_render_core_example)
add_subdirectory(model_reconstruction)