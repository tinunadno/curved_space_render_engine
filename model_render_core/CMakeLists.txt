cmake_minimum_required(VERSION 3.15)

project(model_render_core LANGUAGES CXX)

add_library(model_render_core INTERFACE)

target_include_directories(model_render_core
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(model_render_core INTERFACE cxx_std_20)

target_compile_definitions(model_render_core
        INTERFACE
)
