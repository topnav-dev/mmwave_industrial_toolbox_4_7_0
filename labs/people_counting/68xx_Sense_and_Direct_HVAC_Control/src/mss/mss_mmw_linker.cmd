/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
    systemHeap : {} > DATA_RAM
    .l2data : {} > DATA_RAM
    .l3data: {} > L3_RAM
    .L2heap   : {} > DATA_RAM ALIGN(32)
    .L2ScratchSect   : {} > DATA_RAM ALIGN(32)
	
}
/*----------------------------------------------------------------------------*/

