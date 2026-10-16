/* STM32F407IG: 1 MB Flash、128 KB DMA 可访问 SRAM。
 * 沿用 Keil 启动文件的 8 KB C 堆和 8 KB 主栈；FreeRTOS 堆仍由原配置分配。
 */
ENTRY(Reset_Handler)
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 120K
    STACK (rw)  : ORIGIN = 0x2001E000, LENGTH = 8K
}
_estack = ORIGIN(STACK) + LENGTH(STACK);
_Min_Heap_Size = 0x2000;
_Min_Stack_Size = 0x2000;

SECTIONS
{
    .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } >FLASH
    .text :
    {
        . = ALIGN(4);
        *(.text .text.*) *(.glue_7 .glue_7t) *(.eh_frame)
        KEEP(*(.init)) KEEP(*(.fini))
        . = ALIGN(4); _etext = .;
    } >FLASH
    .rodata : { . = ALIGN(4); *(.rodata .rodata.*) . = ALIGN(4); } >FLASH
    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM.exidx :
    {
        __exidx_start = .; *(.ARM.exidx*) __exidx_end = .;
    } >FLASH
    .preinit_array (READONLY) :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH
    .init_array (READONLY) :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH
    .fini_array (READONLY) :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4); _sdata = .;
        *(.data .data.*) *(.RamFunc .RamFunc.*)
        . = ALIGN(4); _edata = .;
    } >RAM AT>FLASH
    .bss (NOLOAD) :
    {
        . = ALIGN(4); _sbss = .; __bss_start__ = .;
        *(.bss .bss.*) *(COMMON)
        . = ALIGN(4); _ebss = .; __bss_end__ = .;
    } >RAM
    .heap (NOLOAD) :
    {
        . = ALIGN(8); end = .; _end = .; __heap_start__ = .;
        . += _Min_Heap_Size; __heap_end__ = .;
    } >RAM
    .stack (_estack - _Min_Stack_Size) (NOLOAD) :
    {
        __stack_limit__ = .;
        . += _Min_Stack_Size;
    } >STACK
    ASSERT(__heap_end__ <= __stack_limit__, "RAM overflow: heap overlaps main stack")
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
