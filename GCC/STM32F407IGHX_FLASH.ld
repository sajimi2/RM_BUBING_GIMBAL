/* STM32F407IGHx：与原 Keil 工程相同的 1 MB Flash、128 KB 普通 SRAM。
 * DMA 缓冲区继续位于普通 SRAM，不自动放入 DMA 无法访问的 CCM。
 * 主栈和 C 库堆各保留 8 KB；FreeRTOS 的 24 KB 堆由 heap_4.c 独立管理。 */
ENTRY(Reset_Handler)
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 120K
    STACK (rw) : ORIGIN = 0x2001E000, LENGTH = 8K
}
_estack = ORIGIN(STACK) + LENGTH(STACK);
__StackLimit = ORIGIN(STACK);

SECTIONS
{
    .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH
    .text :
    {
        . = ALIGN(4);
        *(.text .text.*) *(.glue_7) *(.glue_7t)
        KEEP(*(.init)) KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH
    .rodata : { . = ALIGN(4); *(.rodata .rodata.*) . = ALIGN(4); } > FLASH
    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH
    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH
    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*) *(.RamFunc .RamFunc.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT> FLASH
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = .;
        *(.bss .bss.*) *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = .;
    } > RAM
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        end = .;
        _end = .;
        __HeapBase = .;
        . += 0x2000;
        __HeapLimit = .;
    } > RAM
    .stack (NOLOAD) : { . += LENGTH(STACK); } > STACK
    ASSERT(__HeapLimit <= __StackLimit, "RAM overflow: C heap overlaps main stack")
    .ARM.attributes 0 : { *(.ARM.attributes) }
    /DISCARD/ : { *(.note.GNU-stack) }
}
