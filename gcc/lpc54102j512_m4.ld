/* Describe the available memory blocks */
MEMORY
{
    flash           : ORIGIN = 0x00008000, LENGTH = 480K-64K
    config          : ORIGIN = 0x00078000, LENGTH = 1K
    reserved1       : ORIGIN = 0x00078400, LENGTH = 3K
    ephems          : ORIGIN = 0x00079000, LENGTH = 8K
    reserved2       : ORIGIN = 0x0007B000, LENGTH = 12K
    ephemsdownload  : ORIGIN = 0x0007E000, LENGTH = 8K

    sram0           : ORIGIN = 0x02000000, LENGTH = 64K
    sram1a          : ORIGIN = 0x02010000, LENGTH = 24K
    sram1b          : ORIGIN = 0x02016000, LENGTH = 8K
    sram2           : ORIGIN = 0x03400000, LENGTH = 8K
}


ENTRY(Reset_Handler)


/* Stack grows downwards from the end of the first (main) RAM block.
 * Keep 32 bytes free for calls to the IAP flash programming functions.
 * Keep another 16 bytes free to work around a GDB issue where it reads
 * beyond the end of the stack.
 */
__stack_end__ = ORIGIN(sram0) + LENGTH(sram0) - 32 - 16;

SECTIONS
{
    . = ORIGIN(flash);

    .text :
    {
        KEEP(*(.vectors))

        *(.text .text.* .rodata .rodata.*)
        *(.ARM.extab*)
        *(.gnu.linkonce.t.*)
        *(.gnu.linkonce.r.*)
    } >flash

    /* Ordered library section goes separately. */
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } >flash


    .data0 :
    {
        *(.pdm_buffer_align2048)
        *(.data .data.*)
    } >sram0 AT>flash

    .ramcode :
    {
        KEEP(*(LPCLIB_RAMCODE))
    } >sram0 AT>flash

    .data1 :
    {
        KEEP(*(.m0code))
        *(.data1)
    } >sram1a AT>flash

    .data2 :
    {
        *(.ezh_text)
        *(.data2)
    } >sram2 AT>flash


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
        LONG(LOADADDR(.data0))
        LONG(ADDR(.data0))
        LONG(SIZEOF(.data0))

        LONG(2)
        LONG(LOADADDR(.data1))
        LONG(ADDR(.data1))
        LONG(SIZEOF(.data1))

        LONG(2)
        LONG(LOADADDR(.data2))
        LONG(ADDR(.data2))
        LONG(SIZEOF(.data2))

        LONG(1)
        LONG(ADDR(.bss0))
        LONG(SIZEOF(.bss0))

        LONG(1)
        LONG(ADDR(.bss1))
        LONG(SIZEOF(.bss1))

        LONG(1)
        LONG(ADDR(.bss1b))
        LONG(SIZEOF(.bss1b))

        LONG(1)
        LONG(ADDR(.bss2))
        LONG(SIZEOF(.bss2))

        LONG(0)
    } >flash

    .noinit :
    {
        __noinit_start__ = .;
        *(.noinit)
        __noinit_end__ = ALIGN(4);
    } >sram0

    __noinit_size__ = SIZEOF(.noinit);

    .bss0 :
    {
        *(.bss .bss.* COMMON)
        __newlib_heap_start__ = ALIGN(4);
        __heap_start = .;
    } >sram0

    .bss1 :
    {
        *(.bss1)
    } >sram1a

    .bss1b (NOLOAD) :
    {
        *(.ipc)
    } >sram1b


    .bss2 :
    {
        *(.ezh_bss)
        *(.bss2)
    } >sram2

    .ephems :
    {
        *(.rinexdata)
    } >ephems

    .config (NOLOAD) :
    {
        KEEP(*(.config))
    } >config

    .ephemsdownload (NOLOAD) :
    {
        KEEP(*(.ephemsdownload))
    } >ephemsdownload


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
                                  + MPU_Handler + 1
                                  + BusFault_Handler + 1
                                  + UsageFault_Handler + 1
                                  ));
}

