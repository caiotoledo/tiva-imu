cmake_minimum_required (VERSION 3.10)
set(PROJECT_NAME uart)
project(${PROJECT_NAME})

include(${CMAKE_TOOLCHAIN_FILE})

set(src_dir "${PROJECT_SOURCE_DIR}/src/")
file(GLOB src_files "${src_dir}/*.c")

set(src_uartstdio "${TIVAWARE_PATH}/utils/uartstdio.c")

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/../include")

add_library(${PROJECT_NAME} STATIC ${src_files} ${src_uartstdio})
target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME}
    ${TIVAWARE_PATH}/driverlib/gcc/libdriver.a
)
