INCLUDE(CMakeForceCompiler)

SET(CMAKE_SYSTEM_NAME Generic)
# SET(CMAKE_SYSTEM_PROCESSOR arm)
# SET(CMAKE_CROSSCOMPILING 1)
# SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}as)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

# set compiler flags
set(CPU "-mcpu=cortex-m4")
set(FPU "-mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_ASM_FLAGS "-mthumb ${CPU} ${FPU} -MD")
set(CMAKE_C_FLAGS "-mthumb ${CPU} ${FPU} -std=gnu99 -Os -ffunction-sections -fdata-sections -fsingle-precision-constant -MD -Wall -pedantic")
set(CMAKE_CXX_FLAGS "-mthumb ${CPU} ${FPU} -Os -ffunction-sections -fdata-sections -fsingle-precision-constant -MD -Wall -pedantic -std=c++11 -fno-exceptions -fno-rtti")

# set linker flags
set(MCU "TM4C123GH6PM")
set(LD_SCRIPT "${MCU}.ld")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "-T${PROJECT_SOURCE_DIR}/etc/${LD_SCRIPT} -specs=${PROJECT_SOURCE_DIR}/etc/tiva.specs")

# add GCC specific definitions
add_definitions(-Dgcc)
