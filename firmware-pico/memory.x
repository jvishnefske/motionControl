/* RP2040 — BTT SKR Pico v1.0 */
MEMORY
{
    /* RP2040 boots from external QSPI flash (2MB on SKR Pico).
       The boot2 bootloader copies code to XIP region at 0x10000000. */
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}
