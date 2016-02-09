SECTIONS
{
    /* Flash configuration field, very important in order to not accidentally lock the device */
    /* Flash configuration field 0x400-0x40f. */
    .fcfield 0x400 :
    {
        /* Backdoor key. */
        LONG (0x01234567) LONG (0x89abcdef)

        /* FPROT0-3: no region of the flash is protected. */
        LONG (0xffffffff)

        /* FSEC: place MCU in unsecure mode. */
        BYTE (0xfe)

        /* FOPT: disable EzPort. */
        BYTE (0xfd)

        /* FEPROT: default. */
        BYTE (0xff)

        /* FDPROT: default. */
        BYTE (0xff)
    } >flashsec
    ASSERT (SIZEOF(.fcfield) == 0x10, "Flash security sector of invalid size (linker-script error?).")
    ASSERT (ADDR(.fcfield) == 0x400, "Flash configuration field at invalid position (linker-script error?)")
    ASSERT (LOADADDR(.fcfield) == 0x400, "Flash configuration field at invalid position (linker-script error?)")
}
