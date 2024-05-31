/******************************************************************************
 *
 * linker_script.ld - Linker script for applications using startup_gnu.c
 *
 *****************************************************************************/

FLASH_START = 0x0000C000;
FLASH_SIZE  = 0x001F4000;
TCM_RAM_START   = 0x10000000;
TCM_RAM_SIZE    = 0x00010000;
STACK_START = 0x10010000
STACK_SIZE  = 0x00002000;
RAM_START   = 0x10012000;
RAM_SIZE    = 0x000AE000;
VECTORS_SIZE = 0x100;

MEMORY
{
    FLASH (rx) : ORIGIN = FLASH_START, LENGTH = FLASH_SIZE
    TCM (rwx) : ORIGIN = TCM_RAM_START, LENGTH = TCM_RAM_SIZE
    STACKMEM (rwx) : ORIGIN = STACK_START, LENGTH = STACK_SIZE
    RAM (rwx) : ORIGIN = RAM_START, LENGTH = RAM_SIZE
}

ENTRY(Reset_Handler)

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        KEEP(*(.patch))
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    
        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.rodata*)

        KEEP(*(.eh_frame*))
    } > FLASH

    .ARM.extab :
    {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    _etext = .;

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        *(vtable)

        . = ALIGN(8);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(8);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);


        . = ALIGN(8);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(8);

        _edata = .;
    } > RAM AT>FLASH

    /* used by startup to initialize data */
    _init_data = LOADADDR(.data);

    .tcm :
    {
        . = ALIGN(4);
        
        PROVIDE(__vectors_start__ = .);
        . += VECTORS_SIZE;
        PROVIDE(__vectors_end__ = .);

        _stcm = .;
        *(.tcm)
        *(.tcm*)
        . = ALIGN(4);
        _etcm = .;
    } > TCM AT>FLASH

    /* used by startup to initialize tcm */
    _init_tcm = LOADADDR(.tcm);

    /* User stack section initialized by startup code. */
    .stack (NOLOAD):
    {
        . = ALIGN(8);
        *(.stack)
        *(.stack*)
        . = ALIGN(8);
    } > STACKMEM

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    .uninitialized (NOLOAD):
    {
        . = ALIGN(32);
        __uninitialized_start = .;
        *(.uninitialized)
        KEEP(*(.keep.uninitialized))
        . = ALIGN(32);
        __uninitialized_end = .;
    } > RAM

    .heap (COPY):
    {
    __end__ = .;
    PROVIDE(end = .);
    __HeapBase = .;
    *(.heap*)
    . = ORIGIN(RAM) + LENGTH(RAM);
    __HeapLimit = .;
    } > RAM
    PROVIDE(HEAP_SIZE = __HeapLimit - __HeapBase);

    .ARM.attributes 0 : { *(.ARM.attributes) }
}