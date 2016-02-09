SECTIONS
{
    /* Interrupt vectors 0x00-0x3ff. */
    .vector_table :
    {
        _vector_rom = .;
        KEEP(*(.vector_table))
    } > vectors
    ASSERT (SIZEOF(.vector_table) == 0x400, "Interrupt vector table of invalid size.")
    ASSERT (ADDR(.vector_table) == 0x00000000, "Interrupt vector table at invalid location (linker-script error?)")
    ASSERT (LOADADDR(.vector_table) == 0x00000000, "Interrupt vector table at invalid location (linker-script error?)")
}
