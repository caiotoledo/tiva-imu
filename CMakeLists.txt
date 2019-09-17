cmake_minimum_required (VERSION 3.10)
set(PROJECT_NAME program)
project(${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../.bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../.bin)

set(src_dir "${PROJECT_SOURCE_DIR}/src/")
file(GLOB_RECURSE src_files "${src_dir}/*.c")

# Disable compiler optimization:
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

include_directories("${PROJECT_SOURCE_DIR}/include")
# Include tivaware folder from Texas Instruments
if ("${TIVAWARE_PATH}" STREQUAL "")
    message(FATAL_ERROR "TIVAWARE_PATH variable not set!")
endif()
include_directories("${TIVAWARE_PATH}")

# Create the binary
add_executable(${PROJECT_NAME} ${src_files})

# Add FreeRTOS library
add_subdirectory(thirdparty/freertos ./build_freertos)
# Add RGB library
add_subdirectory(thirdparty/rgb ./build_rgb)
# Add I2C library
add_subdirectory(thirdparty/i2c ./build_i2c)
# Add MPU6050 library
add_subdirectory(thirdparty/mpu6050 ./build_mpu6050)
# Add Button library
add_subdirectory(thirdparty/button ./build_button)
# Add UART library
add_subdirectory(thirdparty/uart ./build_uart)

# Link all libraries
target_link_libraries(${PROJECT_NAME} freertos rgb mpu6050 button uart)

# define objcopy macro
macro(OBJCOPY_FILE EXE_NAME)
    set(FO ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${EXE_NAME}.bin)
    set(FI ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${EXE_NAME})
    message(STATUS ${FO})
    add_custom_command(
        OUTPUT ${FO}
        COMMAND ${CMAKE_OBJCOPY}
        ARGS -O binary -I elf32-little ${FI} ${FO}
        DEPENDS ${FI}
    )
    get_filename_component(TGT "${EXE_NAME}" NAME)
    add_custom_target("target-objcopy_${TGT}" ALL DEPENDS ${FO} VERBATIM)
    get_directory_property(extra_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    set_directory_properties(
        PROPERTIES
        ADDITIONAL_MAKE_CLEAN_FILES "${extra_clean_files};${FO}"
    )
    set_source_files_properties("${FO}" PROPERTIES GENERATED TRUE)
endmacro(OBJCOPY_FILE)

# set the objcopy for binary file
objcopy_file(${PROJECT_NAME})