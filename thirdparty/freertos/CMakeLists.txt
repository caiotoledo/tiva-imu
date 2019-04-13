cmake_minimum_required (VERSION 3.10)
set(PROJECT_NAME freertos)
project(${PROJECT_NAME})

include(../../etc/toolchain_tivaware.cmake)

set(src_dir "${PROJECT_SOURCE_DIR}/FreeRTOS/Source/")
file(GLOB src_files "${src_dir}/*.c")
file(GLOB src_port "${src_dir}/portable/GCC/ARM_CM4F/*.c")
set(src_heap "${src_dir}/portable/MemMang/heap_2.c")

include_directories("${PROJECT_SOURCE_DIR}/FreeRTOS/Source/include")
include_directories("${PROJECT_SOURCE_DIR}/FreeRTOS/Source/portable/GCC/ARM_CM4F")

add_library(${PROJECT_NAME} STATIC ${src_files} ${src_port} ${src_heap})
target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR}/FreeRTOS/Source/include)
target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR}/FreeRTOS/Source/portable/GCC/ARM_CM4F)
