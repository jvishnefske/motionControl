/* Linker script for ATSAME54P20A — Duet 3 Mini 5+ */
/* Cortex-M4F: 1MB Flash, 256KB SRAM                */
MEMORY
{
    FLASH : ORIGIN = 0x00004000, LENGTH = 1008K  /* 16K reserved for bootloader */
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
