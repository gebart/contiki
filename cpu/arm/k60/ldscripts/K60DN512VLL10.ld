OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

MEMORY
{
    vectors  (rx)  : ORIGIN = 0x00000000,             LENGTH = 0x400
    flashsec (rx)  : ORIGIN = 0x00000400,             LENGTH = 0x10
    rom      (rx)  : ORIGIN = 0x00000410,             LENGTH = 512K - 0x410
    ram      (rwx) : ORIGIN = 0x20000000 - 64K,       LENGTH = 128K - 2K
    stack    (rw)  : ORIGIN = 0x20000000 + 64K - 2K,  LENGTH = 2K
}

/* Interrupt vectors */
INCLUDE vector-table.ld

/* Flash security section, very important in order to not accidentally lock the device */
INCLUDE flash-security.ld

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
