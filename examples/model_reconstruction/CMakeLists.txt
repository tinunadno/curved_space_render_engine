cmake_minimum_required(VERSION 3.15)

project(model_reconstruction LANGUAGES CXX)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(manifold REQUIRED)

add_executable(model_reconstruction
        main.cpp
        snapshot.h
        model_intersection.h
        model_intersection.cpp
)

target_compile_definitions(model_reconstruction PRIVATE PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(model_reconstruction
        PRIVATE
        engine_core
        model_render_core
        glfw
        OpenGL::GL
        manifold::manifold
)
