###################################################################################
# Millimeter Wave Demo
###################################################################################
.PHONY: mssDemo mssDemoClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c ./SDK_adaptation/demo_utils \
vpath %.c ./SDK_adaptation/objdetrangehwaDPC/src \
vpath %.c ./SDK_adaptation/rangeprochwaDPU/src \
vpath %.c ./SDK_adaptation/trackerproc/src \
          ./mss

PLATFORM_R4F_LINK_CMD   = $(MMWAVE_DEMO_DEV_PATH)/src/chains/PeopleCounting3D/demo/mss/r4f_linker.cmd

###################################################################################
# Additional libraries which are required to build the DEMO:
###################################################################################
MSS_PCOUNT3DDEMO_STD_LIBS = $(R4F_COMMON_STD_LIB)						\
			-llibpinmux_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT) 		\
		   	-llibcrc_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
		   	-llibuart_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
		   	-llibmailbox_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
		   	-llibmmwavelink_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)	\
		   	-llibmmwave_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
		   	-llibadcbuf_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)      	\
		   	-llibdma_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)         	\
		   	-llibgpio_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)         	\
		   	-llibedma_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
		   	-llibcli_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
            -llibdpm_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT) 		\
            -llibmathutils.$(R4F_LIB_EXT) 					\
			-llibhwa_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)           	\
			-llibdpedma_hwa_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)        \
            -llibosal_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)       \
            -llibcbuff_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)         \
            -llibhsiheader_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)            \
            -llibgtrack3D.$(R4F_LIB_EXT)

MSS_PCOUNT3DDEMO_LOC_LIBS = $(R4F_COMMON_LOC_LIB)					\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/pinmux/lib 	\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/uart/lib	\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/crc/lib		\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/mailbox/lib	\
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/adcbuf/lib	\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/dma/lib         \
			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/hwa/lib         \
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/gpio/lib        \
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/edma/lib	\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwavelink/lib	\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/cli/lib		\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwave/lib      \
	        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/dpm/lib         \
            -i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/mathutils/lib     \
			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/datapath/dpedma/lib 	\
            -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/osal/lib       \
            -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/cbuff/lib       \
            -i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/hsiheader/lib     \
            -i$(MMWAVE_DEMO_DEV_PATH)/src/chains/PeopleCounting3D/demo/SDK_adaptation/trackerproc/lib \
	    -i$(MMWAVE_DEMO_DEV_PATH)/src/chains/PeopleCounting3D/demo/SDK_adaptation/gtrack/lib


###################################################################################
# Millimeter Wave Demo
###################################################################################
MSS_MMW_CFG_PREFIX       = pcount3D_mss
MSS_PCOUNT3DDEMO_CFG         = $(MSS_MMW_CFG_PREFIX).cfg
MSS_PCOUNT3DDEMO_ROV_XS      = $(MSS_MMW_CFG_PREFIX)_$(R4F_XS_SUFFIX).rov.xs
MSS_PCOUNT3DDEMO_CONFIGPKG   = mmw_configPkg_mss_$(MMWAVE_SDK_DEVICE_TYPE)
MSS_PCOUNT3DDEMO_MAP         = $(MMWAVE_SDK_DEVICE_TYPE)_Pcount3DDemo_mss.map
MSS_PCOUNT3DDEMO_OUT         = $(MMWAVE_SDK_DEVICE_TYPE)_Pcount3DDemo_mss.$(R4F_EXE_EXT)
MSS_PCOUNT3DDEMO_METAIMG_BIN = $(MMWAVE_SDK_DEVICE_TYPE)_Pcount3DDemo.bin
MSS_PCOUNT3DDEMO_CMD         = mss/pcount3D_mss_linker.cmd
MSS_PCOUNT3DDEMO_SOURCES     = \
                       mmwdemo_rfparser.c  \
                       mmwdemo_adcconfig.c \
                       mss_main.c \
                       pcount3D_cli.c \
                       rangeprochwa.c \
                       objdetrangehwa.c \
 					   trackerproc_3d.c \
                       tracker_utils.c


MSS_PCOUNT3DDEMO_DEPENDS   = $(addprefix $(PLATFORM_OBJDIR)/, $(MSS_PCOUNT3DDEMO_SOURCES:.c=.$(R4F_DEP_EXT)))
MSS_PCOUNT3DDEMO_OBJECTS   = $(addprefix $(PLATFORM_OBJDIR)/, $(MSS_PCOUNT3DDEMO_SOURCES:.c=.$(R4F_OBJ_EXT)))

###################################################################################
# RTSC Configuration:
###################################################################################
mmwMssRTSC:
	@echo 'Configuring RTSC packages...'
	$(XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(R4F_XSFLAGS) -o $(MSS_PCOUNT3DDEMO_CONFIGPKG) mss/$(MSS_PCOUNT3DDEMO_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

###################################################################################
# Build the Millimeter Wave Demo
###################################################################################
mssDemo: BUILD_CONFIGPKG=$(MSS_PCOUNT3DDEMO_CONFIGPKG)
mssDemo: R4F_CFLAGS += --cmd_file=$(BUILD_CONFIGPKG)/compiler.opt \
                       --define=APP_RESOURCE_FILE="<$(MMWAVE_DEMO_DEV_PATH)/src/chains/PeopleCounting3D/demo/pcount3D_hwres.h>" \
                       --c99 -i$(MMWAVE_DEMO_DEV_PATH)/src -dPLATFORM$(SUPPORTPLATFORM) -dTRACKERPROC_EN -dGTRACK_3D  \
                       -i$(MMWAVE_DEMO_DEV_PATH)/src/api/notarget \
		       -i$(MMWAVE_DEMO_DEV_PATH)/src/chains/PeopleCounting3D/demo/SDK_adaptation/gtrack \
		       -i$(MMWAVE_DEMO_DEV_PATH)/src \
                       --define=DebugP_LOG_ENABLED
					   
mssDemo: buildDirectories mmwMssRTSC $(MSS_PCOUNT3DDEMO_OBJECTS)
	$(R4F_LD) $(R4F_LDFLAGS) $(MSS_PCOUNT3DDEMO_LOC_LIBS) $(MSS_PCOUNT3DDEMO_STD_LIBS) 			\
	-l$(MSS_PCOUNT3DDEMO_CONFIGPKG)/linker.cmd --map_file=$(MSS_PCOUNT3DDEMO_MAP) $(MSS_PCOUNT3DDEMO_OBJECTS) 	\
	$(PLATFORM_R4F_LINK_CMD) $(MSS_PCOUNT3DDEMO_CMD) $(R4F_LD_RTS_FLAGS) -o $(MSS_PCOUNT3DDEMO_OUT)
	$(COPY_CMD) $(MSS_PCOUNT3DDEMO_CONFIGPKG)/package/cfg/$(MSS_PCOUNT3DDEMO_ROV_XS) $(MSS_PCOUNT3DDEMO_ROV_XS)
	@echo '******************************************************************************'
	@echo 'Built the MSS for Millimeter Wave Demo'
	@echo '******************************************************************************'

###################################################################################
# Cleanup the Millimeter Wave Demo
###################################################################################
mssDemoClean:
	@echo 'Cleaning the Millimeter Wave Demo MSS Objects'
	@rm -f $(MSS_PCOUNT3DDEMO_OBJECTS) $(MSS_PCOUNT3DDEMO_MAP) $(MSS_PCOUNT3DDEMO_OUT) $(MSS_PCOUNT3DDEMO_METAIMG_BIN) $(MSS_PCOUNT3DDEMO_DEPENDS) $(MSS_PCOUNT3DDEMO_ROV_XS)
	@echo 'Cleaning the Millimeter Wave Demo MSS RTSC package'
	@$(DEL) $(MSS_PCOUNT3DDEMO_CONFIGPKG)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(MSS_PCOUNT3DDEMO_DEPENDS)

