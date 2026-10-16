set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ARM_GCC_PATH 可指向工具链 bin 目录；未设置时从 PATH 查找。
find_program(ARM_GCC arm-none-eabi-gcc HINTS "$ENV{ARM_GCC_PATH}" REQUIRED)
get_filename_component(ARM_GCC_BIN "${ARM_GCC}" DIRECTORY)
set(CMAKE_C_COMPILER "${ARM_GCC}")
set(CMAKE_ASM_COMPILER "${ARM_GCC}")
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy HINTS "${ARM_GCC_BIN}" REQUIRED)
find_program(CMAKE_SIZE arm-none-eabi-size HINTS "${ARM_GCC_BIN}" REQUIRED)

set(CMAKE_C_FLAGS_INIT "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_ASM_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_C_FLAGS_INIT}")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3" CACHE STRING "Debug compiler flags")
set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG" CACHE STRING "Release compiler flags")
