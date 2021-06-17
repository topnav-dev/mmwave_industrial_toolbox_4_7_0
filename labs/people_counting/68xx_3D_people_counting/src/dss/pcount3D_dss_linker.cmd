/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Include Libraries */
-llibdpm_xwr68xx.ae674
-llibmailbox_xwr68xx.ae674
-llibsoc_xwr68xx.ae674
-llibosal_xwr68xx.ae674
-ldsplib.ae674
//-opcount3D_dss_pe674.oe674
/* Section Configuration                                                      */
SECTIONS
{
    systemHeap : {} >> L2SRAM_UMAP0 | L2SRAM_UMAP1
    //.l3ram: {} >> L3SRAM
    .dpc_l1Heap  : { } > L1DSRAM
    .dpc_l2Heap: { } >> L2SRAM_UMAP0 | L2SRAM_UMAP1
	.ovly > L2SRAM_UMAP0 | L2SRAM_UMAP1
	
    /* L3SRAM has code that is overlaid with data, so data must be
       marked uninitialized. Application can initialize this section
       using _L3data_* symbols defined below. Code should be written carefully as
       these are linker symbols (see for example http://e2e.ti.com/support/development_tools/compiler/f/343/t/92002 ):
        
        extern far uint8_t _L3data_start; // the type here does not matter
        extern far uint8_t _L3data_size;  // the type here does not matter

        memset((void *)_symval(&_L3data_start), 0, (uint32_t) _symval(&_L3data_size));
    */ 
    .l3data: type=NOINIT, start(_L3data_start), size(_L3data_size), load=L3SRAM PAGE 1
	
    .fastCode:
    {
		RADARDEMO_detectionCFAR_priv.oe674 (.text:RADARDEMO_detectionCFAR_raCAAll)
		RADARDEMO_aoaEst2DCaponBF_heatmapEst.oe674 (.text:RADARDEMO_aoaEst2DCaponBF_raHeatmap)
		RADARDEMO_aoaEst2DCaponBF_rnEstInv.oe674 (.text:RADARDEMO_aoaEst2DCaponBF_covInv)
		MATRIX_cholesky.oe674 (.text:MATRIX_cholesky_flp_inv)
		RADARDEMO_aoaEst2DCaponBF_staticRemoval.oe674 (.text:RADARDEMO_aoaEst2DCaponBF_clutterRemoval)
		copyTranspose.oe674 (.text:copyTranspose)
		//dsplib.ae674<*.obj>(.text) 
    } load=L3SRAM PAGE 0 , run=L1PSRAM PAGE 0, table(_pcount3DDemo_fastCode_L1PSRAM_copy_table, compression=off)
	

    .hsramCode:
    {
		libdpm_xwr68xx.ae674 (.text:DPM_deinit)
		libmailbox_xwr68xx.ae674 (.text:Mailbox_close)
		libdpm_xwr68xx.ae674 (.text:DPM_pipeDeinit)
		dss_main.oe674 (.text:Pcount3DDemo_sensorStopEpilog)
		
		rts*.lib (.text:_outc)
		rts*.lib (.text:_outs)
		rts*.lib (.text:printf)
		rts*.lib (.text:_ltostr)
		rts*.lib (.text:__c6xabi_isnan)
		rts*.lib (.text:_ecpy)
		rts*.lib (.text:_mcpy)
		rts*.lib (.text:_pconv_g)
		rts*.lib (.text:fcvt)
		rts*.lib (.text:_pconv_f)
		rts*.lib (.text:_pconv_e)
		rts*.lib (.text:_pconv_a)
		rts*.lib (.text:__TI_printfi)
		rts*.lib (.text:fputs)
		rts*.lib (.text:fputc)
		rts*.lib (.text:__c6xabi_divul)
		rts*.lib (.text:__c6xabi_divd)
		rts*.lib (.text:frexp)
		rts*.lib (.text:ldexp)
		
    } load=L3SRAM PAGE 0, run=HSRAM PAGE 0, table(_pcount3DDemo_configCode_HSRAM_copy_table, compression=off)

    .overlaidCode:
    {
    	RADARDEMO_aoaEst2DCaponBF.oe674 (.text:RADARDEMO_aoaEst2DCaponBF_create)
    	RADARDEMO_detectionCFAR.oe674 (.text:RADARDEMO_detectionCFAR_create)
    	RADARDEMO_aoaEst2DCaponBF_utils.oe674 (.text:tw_gen_float)
    	radarProcess.oe674 (.text:DPU_radarProcess_init)
		objectdetection.oe674 (.text:DPC_ObjectDetection_deinit)
		radarOsal_malloc.oe674 (.text:radarOsal_memAlloc)
		radarOsal_malloc.oe674 (.text:radarOsal_memInit)
    	radarOsal_malloc.oe674 (.text:radarOsal_memDeInit)
		radarOsal_malloc.oe674 (.text:radarOsal_printHeapStats)
		objectdetection.oe674 (.text:DPC_ObjectDetection_init)
		RADARDEMO_aoaEst2DCaponBF_utils.oe674 (.text:cosdp_i)
		

    	RADARDEMO_aoaEst2DCaponBF_angleEst.oe674 (.text:RADARDEMO_aoaEst2DCaponBF_aeEstElevAzim)
		libedma_xwr68xx.ae674<*.oe674>(.text)
		dss_main.oe674 (.text:MmwDemo_edmaOpen)
		dss_main.oe674 (.text:MmwDemo_edmaInit)
		dss_main.oe674 (.text:MmwDemo_edmaBlockCopy)
		dss_main.oe674 (.text:MmwDemo_copyTable)
		cycle_measure.oe674 (.text:cache_setMar)
		dss_main.oe674 (.text:MmwDemo_EDMA_errorCallbackFxn)
		dss_main.oe674 (.text:MmwDemo_EDMA_transferControllerErrorCallbackFxn)
		
		libedma_xwr68xx.ae674 (.far:EDMA_object)

		dss_main.oe674 (.text:main)
		dss_main.oe674 (.text:Pcount3DDemo_dssInitTask)
    } load=L3SRAM PAGE 0 

    .unUsedCode:
    {
    	RADARDEMO_detectionCFAR_priv.oe674 (.text:RADARDEMO_detectionCFAR_CAAll)
    	RADARDEMO_detectionCFAR_priv.oe674 (.text:RADARDEMO_detectionCFAR_OS)
    } load=L3SRAM PAGE 0
	
    .slowCode:
    {
		libmailbox_xwr68xx.ae674 (.text:Mailbox_init)
		libdpm_xwr68xx.ae674 (.text:DPM_init)
		libmailbox_xwr68xx.ae674 (.text:Mailbox_open)
		libsoc_xwr68xx.ae674 (.text:SOC_deviceInit)
		libdpm_xwr68xx.ae674 (.text:DPM_mboxInit)
		libosal_xwr68xx.ae674 (.text:SemaphoreP_create)
		libdpm_xwr68xx.ae674 (.text:DPM_pipeInit)
		libsoc_xwr68xx.ae674 (.text:SOC_init)

		objectdetection.oe674 (.text:DPC_ObjDetDSP_preStartConfig)
		
		//pcount3D_dss_pe674.oe674 (.text:xdc_runtime_System_printfExtend__I)
		
		//rts*.lib (.text:__TI_tls_init) //not copied to HSRAM, but moved to L3
		rts*.lib (.text:__c6xabi_divf) 
		rts*.lib (.text:setvbuf)
		rts*.lib (.text:HOSTrename)
		rts*.lib (.text:getdevice)
		rts*.lib (.text:__TI_closefile) 
		rts*.lib (.text:atoi)
		rts*.lib (.text:fflush)
		rts*.lib (.text:fseek)  
		rts*.lib (.text:HOSTlseek)
		rts*.lib (.text:HOSTopen)
		rts*.lib (.text:HOSTwrite)
		rts*.lib (.text:__TI_ltoa)  
		rts*.lib (.text:__TI_wrt_ok)
		rts*.lib (.text:close)
		rts*.lib (.text:HOSTread)
		rts*.lib (.text:HOSTunlink)  
		rts*.lib (.text:__TI_doflush)
		rts*.lib (.text:__divu)
		rts*.lib (.text:modf)
		rts*.lib (.text:HOSTclose)  

		rts*.lib (.text:__TI_cleanup)
		rts*.lib (.text:__c6xabi_fixfu)
		rts*.lib (.text:__remu)
		rts*.lib (.text:finddevice)
		rts*.lib (.text:__TI_readmsg)
		rts*.lib (.text:__c6xabi_fixdu)
		rts*.lib (.text:__c6xabi_llshl)
		rts*.lib (.text:unlink)
		rts*.lib (.text:__TI_writemsg)
		rts*.lib (.text:__c6xabi_llshru)
		rts*.lib (.text:_subcull)
		rts*.lib (.text:lseek)
		rts*.lib (.text:write)
		rts*.lib (.text:__TI_frcmpyd_div)
		rts*.lib (.text:__c6xabi_isinf)
		rts*.lib (.text:wcslen)
    } load=L3SRAM PAGE 0 (HIGH)
	
}
/*----------------------------------------------------------------------------*/

