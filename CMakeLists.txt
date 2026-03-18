cmake_minimum_required(VERSION 3.20)

project(pc_radar_algo_float_v2 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(pc_radar_algo_float_v2
    src/arm_math_compat.c
    src/pc_radar_debug.c
    src/main.c
    src/pc_radar_pipeline.c
    src/radar_hook.c
    src/algorithm/base/source/radar_cfar.c
    src/algorithm/base/source/radar_dbf.c
    src/algorithm/gesture/source/radar_svm.c
)

target_include_directories(pc_radar_algo_float_v2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/base/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/gesture/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/gesture
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/gesture/source
)

if (MSVC)
    target_compile_options(pc_radar_algo_float_v2 PRIVATE /W3)
else()
    target_compile_options(pc_radar_algo_float_v2 PRIVATE -Wall -Wextra)
    target_link_libraries(pc_radar_algo_float_v2 PRIVATE m)
endif()

