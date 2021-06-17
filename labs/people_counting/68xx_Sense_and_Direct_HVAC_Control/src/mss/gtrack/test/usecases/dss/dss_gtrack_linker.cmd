/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

-stack 0x1000

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
    systemHeap : {} >> L2SRAM_UMAP0 | L2SRAM_UMAP1
	.gtrackLibSection {
		_gtrackLibStart = .;
		lib\libgtrack*.ae674 (.text)
		lib\libgtrack*.ae674 (.const)
		_gtrackLibEnd = .;
	} > L2SRAM_UMAP1
	
	.MCPILogBuffer : {} > L3SRAM	
}
/*----------------------------------------------------------------------------*/

