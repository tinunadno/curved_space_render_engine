cmake_minimum_required(VERSION 3.15)

project(shader_core LANGUAGES CXX)

add_library(shader_core INTERFACE
        ../model_render_core_example/main.cpp)

target_include_directories(shader_core
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(shader_core INTERFACE cxx_std_20)

target_compile_definitions(shader_core
        INTERFACE ENABLE_LOG
)
