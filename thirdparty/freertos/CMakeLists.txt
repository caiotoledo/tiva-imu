cmake_minimum_required (VERSION 3.10)
set(PROJECT_NAME freertos)
project(${PROJECT_NAME})

include(${CMAKE_TOOLCHAIN_FILE})

# Path to freertos source and header files
set(freertos_dir "${PROJECT_SOURCE_DIR}/FreeRTOS/FreeRTOS/")

set(src_dir "${freertos_dir}/Source/")
file(GLOB src_files "${src_dir}/*.c")
file(GLOB src_port "${src_dir}/portable/GCC/ARM_CM4F/*.c")
set(src_heap "${src_dir}/portable/MemMang/heap_2.c")

add_library(${PROJECT_NAME} STATIC ${src_files} ${src_port} ${src_heap})
target_include_directories(${PROJECT_NAME} PUBLIC ${freertos_dir}/Source/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${freertos_dir}/Source/portable/GCC/ARM_CM4F)
