cmake_minimum_required(VERSION 3.15)

project(model_render_core LANGUAGES CXX)

add_library(model_render_core INTERFACE
        include/model/model_geometry.h
        include/model/model.h
        include/model/texture.h
        include/model/material.h
        include/model/io.h
        include/main_pipeline.h
        include/utils/point_projection.h
        include/utils/graphics_tools.h
        include/utils/compute_normals.h
        include/utils/vertices_transform.h
        include/scene_cache.h
        include/light_source.h
)

target_include_directories(model_render_core
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(model_render_core INTERFACE cxx_std_20)

target_compile_definitions(model_render_core
        INTERFACE
)
