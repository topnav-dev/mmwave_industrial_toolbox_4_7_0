-llibtrackerproc_xwr68xx.ae674
/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
    systemHeap : {} >> L2SRAM_UMAP0 | L2SRAM_UMAP1
    .l3ram: {} >> L3SRAM
    .dpc_l1Heap  : { } > L1DSRAM
    .dpc_l2Heap: { } >> L2SRAM_UMAP0 | L2SRAM_UMAP1
    .demoSharedMem: { } >> HSRAM

#if (TRACKERDPU_ON_DSP)
    .l3code: 
    {
        libtrackerproc_xwr68xx.ae674(.text)
        dss_main.oe674 (.text:main)
        dss_main.oe674 (.text:MmwDemo_edmaInit)
        dss_main.oe674 (.text:MmwDemo_edmaOpen)
        dss_main.oe674 (.text:MmwDemo_EDMA_errorCallbackFxn)
        dss_main.oe674 (.text:MmwDemo_EDMA_transferControllerErrorCallbackFxn)
        dss_main.oe674 (.text:MmwDemo_edmaClose)
        dss_main.oe674 (.text:MmwDemo_sensorStopEpilog)
        /*dss_main.oe674 (.text:MmwDemo_DPC_ObjectDetection_reportFxn)
        dss_main.oe674 (.text:MmwDemo_updateObjectDetStats)
        dss_main.oe674 (.text:MmwDemo_copyResultToHSRAM)
        dss_main.oe674 (.text:MmwDemo_DPC_ObjectDetection_dpmTask)*/
        dss_main.oe674 (.text:MmwDemo_dssInitTask)

        /*objectdetection.oe674 (.text:DPC_ObjDetDSP_MemPoolReset)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_MemPoolSet)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_MemPoolGet)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_MemPoolGetMaxUsage)*/
        objectdetection.oe674 (.text:_DPC_Objdet_Assert)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_rangeConfig)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_dopplerConfig)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_CFARCAconfig)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_reconfigSubFrame)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_AoAconfig)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_initDPU)
        objectdetection.oe674 (.text:DPC_ObjDetDSP_deinitDPU)
   } >> L3SRAM_CODE
#endif

}
/*----------------------------------------------------------------------------*/

