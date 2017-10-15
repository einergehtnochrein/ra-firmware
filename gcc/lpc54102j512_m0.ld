/* Describe the available memory blocks */
MEMORY
{
    sram0           : ORIGIN = 0x02000000, LENGTH = 64K
    sram1a          : ORIGIN = 0x02010000, LENGTH = 24K
    sram1b          : ORIGIN = 0x02016000, LENGTH = 8K
    sram2reserved   : ORIGIN = 0x03400000, LENGTH = 6K      /* Used for EZH code at run-time */
    sram2           : ORIGIN = 0x03401000, LENGTH = 2K
}


ENTRY(Reset_Handler)


/* Stack grows downwards from the end of the first (main) RAM block.
 * Keep 16 bytes free to work around a GDB issue where it reads
 * beyond the end of the stack.
 */
__stack_end__ = ORIGIN(sram1a) + LENGTH(sram1a) - 16;

SECTIONS
{
    . = ORIGIN(sram1a);

    .text :
    {
        KEEP(*(.vectors))

        *(.text .text.* .rodata .rodata.*)
        *(.ARM.extab*)
        *(.gnu.linkonce.t.*)
        *(.gnu.linkonce.r.*)
    } >sram1a

    /* Ordered library section goes separately. */
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } >sram1a


    .data1 :
    {
        *(.data .data.*)
    } >sram1a

    .ramcode :
    {
        KEEP(*(LPCLIB_RAMCODE))
    } >sram1a


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

        LONG(1)
        LONG(ADDR(.bss1))
        LONG(SIZEOF(.bss1))

        LONG(0)
    } >sram1a

    .noinit :
    {
        __noinit_start__ = .;
        *(.noinit)
        __noinit_end__ = ALIGN(4);
    } >sram1a

    __noinit_size__ = SIZEOF(.noinit);

    .bss1 :
    {
        *(.bss .bss.* COMMON)
        __newlib_heap_start__ = ALIGN(4);
        __heap_start = .;
    } >sram1a

    .bss1b :
    {
        *(.ipc)
    } >sram1b


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

