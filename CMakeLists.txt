cmake_minimum_required(VERSION 3.21)
project(standard_robot LANGUAGES C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU" OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    message(FATAL_ERROR "请使用 CMakePresets.json 中的 Debug/Release 预设和 ARM GCC 工具链")
endif()

include(cmake/sources.cmake)
set(DSP_SOURCE_DIR Middlewares/ST/ARM/DSP/Source)
add_executable(standard_robot
    ${FIRMWARE_SOURCES}
    GCC/startup_stm32f407xx.s
    GCC/syscalls.c
    GCC/layout_checks.c
    ${DSP_SOURCE_DIR}/MatrixFunctions/arm_mat_init_f32.c
    ${DSP_SOURCE_DIR}/MatrixFunctions/arm_mat_add_f32.c
    ${DSP_SOURCE_DIR}/MatrixFunctions/arm_mat_sub_f32.c
    ${DSP_SOURCE_DIR}/MatrixFunctions/arm_mat_mult_f32.c
    ${DSP_SOURCE_DIR}/MatrixFunctions/arm_mat_trans_f32.c
    ${DSP_SOURCE_DIR}/MatrixFunctions/arm_mat_inverse_f32.c
    ${DSP_SOURCE_DIR}/FastMathFunctions/arm_sin_f32.c
    ${DSP_SOURCE_DIR}/FastMathFunctions/arm_cos_f32.c
    ${DSP_SOURCE_DIR}/CommonTables/arm_common_tables.c
)
set_target_properties(standard_robot PROPERTIES SUFFIX ".elf")
target_include_directories(standard_robot PRIVATE ${FIRMWARE_INCLUDE_DIRS})
target_compile_definitions(standard_robot PRIVATE USE_HAL_DRIVER STM32F407xx ARM_MATH_CM4)
# 保留 ARMCC 的无符号 char、小枚举 ABI；旧算法含位重解释，不启用严格别名优化。
target_compile_options(standard_robot PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-ffunction-sections;-fdata-sections;-funsigned-char;-fshort-enums;-fno-strict-aliasing;-ffp-contract=off;-Wall;-Wextra;-Wno-unused-parameter;-Werror=implicit-function-declaration>
)
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/GCC/STM32F407IGHX_FLASH.ld")
set_property(TARGET standard_robot APPEND PROPERTY LINK_DEPENDS "${LINKER_SCRIPT}")
target_link_options(standard_robot PRIVATE
    "-T${LINKER_SCRIPT}" --specs=nano.specs --specs=nosys.specs -nostartfiles
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/standard_robot.map"
    -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-u,_printf_float
)
target_link_libraries(standard_robot PRIVATE m c)
add_custom_command(TARGET standard_robot POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:standard_robot>" "${CMAKE_CURRENT_BINARY_DIR}/standard_robot.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:standard_robot>" "${CMAKE_CURRENT_BINARY_DIR}/standard_robot.bin"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:standard_robot>"
    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/standard_robot.hex" "${CMAKE_CURRENT_BINARY_DIR}/standard_robot.bin"
    VERBATIM
)

# flash 显式执行才写板；先完成依赖构建，再由 CubeProgrammer 下载、校验、复位。
if(WIN32 OR CMAKE_HOST_WIN32)
    find_program(POWERSHELL_EXECUTABLE NAMES powershell.exe pwsh REQUIRED)
    add_custom_target(flash
        COMMAND "${POWERSHELL_EXECUTABLE}" -NoProfile -ExecutionPolicy Bypass
                -File "${CMAKE_CURRENT_SOURCE_DIR}/tools/flash.ps1"
                -Firmware "$<TARGET_FILE:standard_robot>"
        DEPENDS standard_robot
        USES_TERMINAL VERBATIM
    )
endif()
