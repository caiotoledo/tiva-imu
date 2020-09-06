cmake_minimum_required (VERSION 3.10)
set(PROJECT_NAME program)
project(${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/.bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/.bin)

set(src_dir "${PROJECT_SOURCE_DIR}/src/")
file(GLOB_RECURSE src_files "${src_dir}/*.c")

# Include more warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

# Disable compiler optimization in debug:
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

# Retrieve current commit hash
execute_process (
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    RESULT_VARIABLE STATUS_CMD
    OUTPUT_VARIABLE COMMIT_HASH
)
if(${STATUS_CMD} EQUAL 0)
    string(STRIP ${COMMIT_HASH} COMMIT_HASH)
    # Add git short commit hash macro
    add_definitions(-DCOMMIT_HASH="${COMMIT_HASH}")
endif()

execute_process (
    COMMAND bash -c "if ! $(git diff-index --quiet HEAD); then echo \"+\"; else echo \"\"; fi"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    RESULT_VARIABLE STATUS_CMD
    OUTPUT_VARIABLE LOCAL_DIRTY
)
if(${STATUS_CMD} EQUAL 0)
    string(STRIP ${LOCAL_DIRTY} LOCAL_DIRTY)
    # Add a mark when there is local changes not committed
    add_definitions(-DLOCAL_DIRTY="${LOCAL_DIRTY}")
endif()

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

if(DEFINED LOG_LEVEL)
    add_definitions(-D${LOG_LEVEL})
endif()

if(CMAKE_CROSSCOMPILING)
    # Include tivaware folder from Texas Instruments
    if ("${TIVAWARE_PATH}" STREQUAL "")
        message(FATAL_ERROR "TIVAWARE_PATH variable not set!")
    endif()
    include_directories("${TIVAWARE_PATH}")

    # Create the binary
    add_executable(${PROJECT_NAME} ${src_files})
    include_directories("${PROJECT_SOURCE_DIR}/include")

    # Add tasks library:
    add_subdirectory(tasks)
    # Link all tasks
    target_link_libraries(${PROJECT_NAME}
        led_task
        imu_task
        cmd_task
    )

    # Add local library:
    add_subdirectory(lib)
    # Link all local libraries
    target_link_libraries(${PROJECT_NAME}
        utils
        rgb
        uart
        mcu
    )

    # Add FreeRTOS library
    add_subdirectory(thirdparty)
    # Link all third party libraries
    target_link_libraries(${PROJECT_NAME}
        freertos
    )

    # set the objcopy for binary file
    objcopy_file(${PROJECT_NAME})
endif(CMAKE_CROSSCOMPILING)

enable_testing()
# Macro for Google Test
macro(package_gtest TESTNAME)
    cmake_parse_arguments(
        INPUT_ARG
        ""
        ""
        "SRC;INCLUDE"
        ${ARGN}
    )
    # Execute only for host compilation
    if(NOT CMAKE_CROSSCOMPILING)
        find_package(GTest REQUIRED)
        set(TEST_PROGRAM test_${TESTNAME})
        message(STATUS "Adding google test: ${TESTNAME}")
        add_executable(${TEST_PROGRAM} ${INPUT_ARG_SRC})
        target_include_directories(${TEST_PROGRAM} PRIVATE ${GTEST_INCLUDE_DIRS} ${INPUT_ARG_INCLUDE})
        target_link_libraries(${TEST_PROGRAM} ${GTEST_LIBRARIES} gtest_main gmock gmock_main pthread)
        # Add test to be run in ctest
        add_test(NAME ${TESTNAME}
            COMMAND ${TEST_PROGRAM}
            WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    endif()
endmacro(package_gtest)

add_subdirectory(lib/timer/tests)
