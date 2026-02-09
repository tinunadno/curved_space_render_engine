cmake_minimum_required(VERSION 3.15)

project(engine_core LANGUAGES CXX)

add_library(engine_core INTERFACE)

target_include_directories(engine_core
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(engine_core INTERFACE cxx_std_20)

target_compile_definitions(engine_core
        INTERFACE
)
