MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0X00020000
}

SECTIONS
{
    /* code */
    .text :
    {
        _text = .;
        /* ensure ISR vectors are not removed by linker */
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        KEEP (*(.init))
        KEEP (*(.fini))
        _etext = .;
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    __begin_data = .;

    /* static data */
    .data : AT(__begin_data)
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
    } > SRAM

    __begin_bss = .;

    /* static uninitialized data */
    .bss : AT(__begin_bss)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    . = ALIGN(4);
    end = .;
    _estack = ORIGIN(SRAM) + LENGTH(SRAM) - 4;

    /*.stack :
    {
        _sstack = .;
        . = . + 1K;
        _estack = .;
    } > SRAM

    . = ALIGN(4);

    .heap :
    {
        _heap_start = .;
        end = .;
        . = . + 1K;
        _heap_end = .;
    } > SRAM*/

}
