OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

MEMORY
{
    rom (rx)       : ORIGIN = 0x40100,                LENGTH = 512K - 0x40100
    ram      (rwx) : ORIGIN = 0x20000000 - 64K,       LENGTH = 128K - 2K
    stack    (rw)  : ORIGIN = 0x20000000 + 64K - 2K,  LENGTH = 2K
}

SECTIONS
{   
    . = ORIGIN(rom);   
	/* Interrupt vectors */
    .vector_table :
    {
        _vector_rom = .;
        KEEP(*(.vector_table))
    } > rom
    
    /*
     * Allocate space for interrupt vector in RAM
     */
    .ramvect :
    {
        . = ALIGN(1024);
        _vector_ram_start = .;
        . = _vector_ram_start + 0x400;
        _vector_ram_end = .;
    } > ram
}

/* .text and .ramcode sections */
INCLUDE text.ld

/* .data and .bss sections */
INCLUDE data.ld

/* .stack section */
INCLUDE stack.ld

/* Any debugging sections */
INCLUDE debugging-sections.ld

SECTIONS
{
    /* XXX: what is the purpose of these sections? */
    .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) KEEP (*(.gnu.attributes)) }
    .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }
    /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) }
}
