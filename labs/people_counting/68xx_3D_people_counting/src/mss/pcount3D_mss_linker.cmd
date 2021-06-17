/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
     systemHeap : {} > DATA_RAM
    .hwaBufs: load = HWA_RAM, type = NOINIT
    .dpcLocalRam: {} > DATA_RAM
    .l3ram: {} >> L3_RAM
    .demoSharedMem: { } >> HS_RAM
}
/*----------------------------------------------------------------------------*/

