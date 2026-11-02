cmake_minimum_required(VERSION 3.22.1)
project(mobile3drecon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(recon SHARED
        core/engine.cpp
        core/keyframe_manager.cpp
        mvs/census.cpp
        mvs/refine.cpp
        mvs/stereo.cpp
        tsdf/voxel_hash.cpp
        tsdf/integrator.cpp
        mesh/mesher.cpp
        tango/tango_recon.cpp
        gl/renderer.cpp
        io/exporter.cpp
        jni/native_bridge.cpp)

target_include_directories(recon PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tango_3d_reconstruction/include)

target_compile_options(recon PRIVATE
        -O3
        -ffast-math
        -fno-exceptions
        -fno-rtti
        -fvisibility=hidden
        -Wall
        -Wno-unused-function)

# Android 15 devices can run with 16 KB pages; the app targets SDK 35, so the
# 64-bit library must be segment-aligned for 16 KB just like the vendored core.
if(ANDROID_ABI STREQUAL "arm64-v8a")
    target_link_options(recon PRIVATE "-Wl,-z,max-page-size=16384")
endif()

find_library(log-lib log)
find_library(android-lib android)
find_library(gles-lib GLESv3)
find_library(egl-lib EGL)
find_library(dl-lib dl)

target_link_libraries(recon PRIVATE
        ${log-lib}
        ${android-lib}
        ${gles-lib}
        ${egl-lib}
        ${dl-lib})
