cmake_minimum_required (VERSION 3.10)
project(${PROJECT_NAME})

include(${CMAKE_TOOLCHAIN_FILE})

# FREERTOS BASE LIBRARY
set(LIBRARY_NAME freertos)

# Path to freertos source and header files
set(freertos_dir "${PROJECT_SOURCE_DIR}/FreeRTOS/FreeRTOS/")

set(src_dir "${freertos_dir}/Source/")
file(GLOB src_files "${src_dir}/*.c")
file(GLOB src_port "${src_dir}/portable/GCC/ARM_CM4F/*.c")
set(src_heap "${src_dir}/portable/MemMang/heap_2.c")

add_library(${LIBRARY_NAME} STATIC ${src_files} ${src_port} ${src_heap})
target_include_directories(${LIBRARY_NAME} PUBLIC ${freertos_dir}/Source/include)
target_include_directories(${LIBRARY_NAME} PUBLIC ${freertos_dir}/Source/portable/GCC/ARM_CM4F)

# FREERTOS-CLI LIBRARY
set(LIBRARY_NAME freertos-cli)

# Path to freertos source and header files
set(freertos_cli_dir "${PROJECT_SOURCE_DIR}/FreeRTOS/FreeRTOS-Plus/Source/FreeRTOS-Plus-CLI/")

file(GLOB src_files "${freertos_cli_dir}/*.c")

add_library(${LIBRARY_NAME} STATIC ${src_files})
target_link_libraries(${LIBRARY_NAME} freertos)
target_include_directories(${LIBRARY_NAME} PUBLIC ${freertos_cli_dir})
