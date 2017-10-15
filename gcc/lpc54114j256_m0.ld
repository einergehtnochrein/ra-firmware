/* Describe the available memory blocks */
MEMORY
{
    sram0           : ORIGIN = 0x20000000, LENGTH = 64K
    sram1           : ORIGIN = 0x20010000, LENGTH = 64K
    sram2           : ORIGIN = 0x20020000, LENGTH = 32K
    sramx           : ORIGIN = 0x04000000, LENGTH = 32K
}


ENTRY(Reset_Handler)


/* Stack grows downwards from the end of the first (main) RAM block.
 * Keep 16 bytes free to work around a GDB issue where it reads
 * beyond the end of the stack.
 */
__stack_end__ = ORIGIN(sram1) + LENGTH(sram1) - 16;

SECTIONS
{
    . = ORIGIN(sram1);

    .text :
    {
        KEEP(*(.vectors))

        *(.text .text.* .rodata .rodata.*)
        *(.ARM.extab*)
        *(.gnu.linkonce.t.*)
        *(.gnu.linkonce.r.*)
    } >sram1

    /* Ordered library section goes separately. */
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } >sram1


    .data1 :
    {
        *(.data .data.*)
    } >sram1

    .ramcode :
    {
        KEEP(*(LPCLIB_RAMCODE))
    } >sram1

    /* The scatter table defines sections to be initialized by the startup code.
     * An entry starts with one of these 32-bit words:
     * 0 = end of table
     * 1 = BSS section (initialize with zero)
     * 2 = DATA section. Initialze with non-zero values from flash.
     *
     * 1: Followed by two words for (1) start address and (b) length in bytes
     * 2. Followed by three words for (1) source address, (2) destination address,
     *    and (3) length in bytes
     */
    .scattertable :
    {
        __scattertable_start__ = .;

        LONG(2)
        LONG(LOADADDR(.data1))
        LONG(ADDR(.data1))
        LONG(SIZEOF(.data1))

        LONG(2)
        LONG(LOADADDR(.data2))
        LONG(ADDR(.data2))
        LONG(SIZEOF(.data2))

        LONG(1)
        LONG(ADDR(.bss1))
        LONG(SIZEOF(.bss1))

        LONG(1)
        LONG(ADDR(.bss2))
        LONG(SIZEOF(.bss2))

        LONG(0)
    } >sram1



    .noinit :
    {
        __noinit_start__ = .;
        *(.noinit)
        __noinit_end__ = ALIGN(4);
    } >sram1

    __noinit_size__ = SIZEOF(.noinit);

    .bss1 :
    {
        *(.bss .bss.* COMMON)
        __newlib_heap_start__ = ALIGN(4);
        __heap_start = .;
    } >sram1

    .data2 :
    {
    } >sram2 AT>sram1

    .bss2 :
    {
        *(.ipc)
    } >sram2

    /* RAM code that's needed before scatter loading */
    Load$$LPCLIB_RAMCODE$$Base = LOADADDR(.ramcode);
    Image$$LPCLIB_RAMCODE$$Base = ADDR(.ramcode);
    Image$$LPCLIB_RAMCODE$$Length = SIZEOF(.ramcode);

    /* The vector checksum at location 0x1C must be chosen such that the sum of the first eight words in
     * flash is zero.
     * NOTE: Function symbols won't include the thumb bit (bit 0) here, Add that manually!
     */
    PROVIDE(__vector_checksum__ = -(__stack_end__
                                  + Reset_Handler + 1
                                  + NMI_Handler + 1
                                  + HardFault_Handler + 1
                                  ));
}

