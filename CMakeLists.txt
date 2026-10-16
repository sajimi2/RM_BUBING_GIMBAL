cmake_minimum_required(VERSION 3.22)
project(standard_robot LANGUAGES C ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(cmake/firmware_sources.cmake)

add_executable(standard_robot ${FIRMWARE_SOURCES})
set_target_properties(standard_robot PROPERTIES SUFFIX ".elf")
target_include_directories(standard_robot PRIVATE ${FIRMWARE_INCLUDE_DIRS})
target_compile_definitions(standard_robot PRIVATE USE_HAL_DRIVER STM32F407xx ARM_MATH_CM4)
target_compile_options(standard_robot PRIVATE
    -Wall -ffunction-sections -fdata-sections
    # 原工程使用短枚举和指针重解释；保留数据布局，避免严格别名优化改变计算。
    $<$<COMPILE_LANGUAGE:C>:-fshort-enums;-fno-strict-aliasing;-ffp-contract=off>
)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32F407IGHx_FLASH.ld")
target_link_options(standard_robot PRIVATE
    "-T${LINKER_SCRIPT}"
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/standard_robot.map"
    -Wl,-u,_printf_float
)
set_property(TARGET standard_robot APPEND PROPERTY LINK_DEPENDS "${LINKER_SCRIPT}")
target_link_libraries(standard_robot PRIVATE m)
add_custom_command(TARGET standard_robot POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:standard_robot> standard_robot.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:standard_robot> standard_robot.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:standard_robot>
    VERBATIM
)
