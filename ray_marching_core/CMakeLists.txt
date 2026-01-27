cmake_minimum_required(VERSION 3.15)

project(ray_marching_core LANGUAGES CXX)

add_library(ray_marching_core INTERFACE)

target_include_directories(ray_marching_core
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(ray_marching_core INTERFACE cxx_std_20)

target_compile_definitions(ray_marching_core
        INTERFACE ENABLE_LOG
)
