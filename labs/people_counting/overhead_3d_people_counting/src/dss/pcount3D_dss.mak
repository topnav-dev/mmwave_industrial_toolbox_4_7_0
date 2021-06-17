###################################################################################
# Millimeter Wave Demo
###################################################################################
.PHONY: dssDemo dssDemoClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c $(MMWAVE_SDK_INSTALL_PATH)/ti/demo/utils \
	      $(C674x_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSPF_sp_fftSPxSP/c674 \
          $(MMWAVE_DEMO_DEV_PATH)/src/chains/PeopleCounting3D/dpc/src \
          $(MMWAVE_DEMO_DEV_PATH)/src/chains/PeopleCounting3D/dpu/src \
	      $(MMWAVE_DEMO_DEV_PATH)/src \
	      $(MMWAVE_DEMO_DEV_PATH)/src/modules/utilities \
	      $(MMWAVE_DEMO_DEV_PATH)/src/modules/postProcessing/matrixFunc/src \
	      $(MMWAVE_DEMO_DEV_PATH)/src/modules/DoA/CaponBF2D/src \
	      $(MMWAVE_DEMO_DEV_PATH)/src/modules/detection/CFAR/src \
	      ./dss

PLATFORM_C674X_LINK_CMD_LOCAL = $(MMWAVE_DEMO_DEV_PATH)/src/chains/PeopleCounting3D/demo/dss/c674x_linker.cmd


###################################################################################
# Additional libraries which are required to build the DEMO:
###################################################################################
DSS_PCOUNT3DDEMO_STD_LIBS = $(C674_COMMON_STD_LIB)						\
		   	-llibcrc_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)		\
		   	-llibmailbox_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)		\
			-ldsplib.ae674							\
			-llibedma_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)		\
			-lmathlib.$(C674_LIB_EXT) 					\
			-llibdpm_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT) 		\
			-llibmathutils.$(C674_LIB_EXT) 					\
			-llibosal_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)		\

DSS_PCOUNT3DDEMO_LOC_LIBS = $(C674_COMMON_LOC_LIB)						\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/crc/lib			\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/mailbox/lib	    	\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/edma/lib		\
            -i$(MMWAVE_SDK_INSTALL_PATH)/ti/alg/mmwavelib/lib 		\
	        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/dpm/lib         	\
        	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/mathutils/lib     	\
			-i$(C674x_MATHLIB_INSTALL_PATH)/packages/ti/mathlib/lib 	\
            -i$(C674x_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/lib 

###################################################################################
# Millimeter Wave Demo
###################################################################################
DSS_MMW_CFG_PREFIX       	 = 	pcount3D_dss
DSS_PCOUNT3DDEMO_CFG         = 	$(DSS_MMW_CFG_PREFIX).cfg
DSS_PCOUNT3DDEMO_ROV_XS      = 	$(DSS_MMW_CFG_PREFIX)_$(C674_XS_SUFFIX).rov.xs
DSS_PCOUNT3DDEMO_CONFIGPKG   = 	mmw_configPkg_dss_$(MMWAVE_SDK_DEVICE_TYPE)
DSS_PCOUNT3DDEMO_MAP         = 	$(MMWAVE_SDK_DEVICE_TYPE)_Pcount3DDemo_dss.map
DSS_PCOUNT3DDEMO_OUT         = 	$(MMWAVE_SDK_DEVICE_TYPE)_Pcount3DDemo_dss.$(C674_EXE_EXT)
DSS_PCOUNT3DDEMO_METAIMG_BIN = 	$(MMWAVE_SDK_DEVICE_TYPE)_Pcount3DDemo.bin
DSS_PCOUNT3DDEMO_CMD         = 	dss/pcount3D_dss_linker.cmd
DSS_PCOUNT3DDEMO_SOURCES     = 	objectdetection.c \
								dss_main.c \
								MATRIX_cholesky.c \
								MATRIX_cholesky_dat.c \
								copyTranspose.c \
								RADARDEMO_aoaEst2DCaponBF.c \
								RADARDEMO_aoaEst2DCaponBF_angleEst.c \
								RADARDEMO_aoaEst2DCaponBF_DopplerEst.c \
								RADARDEMO_aoaEst2DCaponBF_heatmapEst.c \
								RADARDEMO_aoaEst2DCaponBF_rnEstInv.c \
								RADARDEMO_aoaEst2DCaponBF_staticHeatMapEst.c \
								RADARDEMO_aoaEst2DCaponBF_staticRemoval.c \
								RADARDEMO_aoaEst2DCaponBF_utils.c \
								RADARDEMO_detectionCFAR.c \
								RADARDEMO_detectionCFAR_priv.c \
								radarOsal_malloc.c \
								radarProcess.c \
								objectdetection.c \
								cycle_measure.c \
								

DSS_PCOUNT3DDEMO_DEPENDS   = $(addprefix $(PLATFORM_OBJDIR)/, $(DSS_PCOUNT3DDEMO_SOURCES:.c=.$(C674_DEP_EXT)))
DSS_PCOUNT3DDEMO_OBJECTS   = $(addprefix $(PLATFORM_OBJDIR)/, $(DSS_PCOUNT3DDEMO_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# RTSC Configuration:
###################################################################################
mmwDssRTSC:
	@echo 'Configuring RTSC packages...'
	$(XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(C674_XSFLAGS) -o $(DSS_PCOUNT3DDEMO_CONFIGPKG) dss/$(DSS_PCOUNT3DDEMO_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

###################################################################################
# Build the Millimeter Wave Demo
###################################################################################
dssDemo: BUILD_CONFIGPKG=$(DSS_PCOUNT3DDEMO_CONFIGPKG)
dssDemo: C674_CFLAGS += --cmd_file=$(BUILD_CONFIGPKG)/compiler.opt --disable_push_pop -k -os --asm_directory="$(DSS_PCOUNT3DDEMO_CONFIGPKG)/package/internal"  \
                        --define=APP_RESOURCE_FILE="<$(MMWAVE_DEMO_DEV_PATH)/src/chains/PeopleCounting3D/demo/pcount3D_hwres.h>" \
                        -i$(C674x_MATHLIB_INSTALL_PATH)/packages \
                        -i$(MMWAVE_DEMO_DEV_PATH)/src \
                        -i$(MMWAVE_DEMO_DEV_PATH)/src/common \
			            -i$(C674x_DSPLIB_INSTALL_PATH)/packages	\
			            --define=DebugP_LOG_ENABLED \

dssDemo: buildDirectories mmwDssRTSC $(DSS_PCOUNT3DDEMO_OBJECTS)
	$(C674_LD) $(C674_LDFLAGS) $(DSS_PCOUNT3DDEMO_LOC_LIBS) $(DSS_PCOUNT3DDEMO_STD_LIBS) 					\
	-l$(DSS_PCOUNT3DDEMO_CONFIGPKG)/linker.cmd --map_file=$(DSS_PCOUNT3DDEMO_MAP) $(DSS_PCOUNT3DDEMO_OBJECTS) 	\
	$(PLATFORM_C674X_LINK_CMD_LOCAL) $(DSS_PCOUNT3DDEMO_CMD) $(C674_LD_RTS_FLAGS) -o $(DSS_PCOUNT3DDEMO_OUT)
	$(COPY_CMD) $(DSS_PCOUNT3DDEMO_CONFIGPKG)/package/cfg/$(DSS_PCOUNT3DDEMO_ROV_XS) $(DSS_PCOUNT3DDEMO_ROV_XS)
	@echo '******************************************************************************'
	@echo 'Built the DSS for Millimeter Wave Demo'
	@echo '******************************************************************************'

###################################################################################
# Cleanup the Millimeter Wave Demo
###################################################################################
dssDemoClean:
	@echo 'Cleaning the Millimeter Wave Demo DSS Objects'
	@rm -f $(DSS_PCOUNT3DDEMO_OBJECTS) $(DSS_PCOUNT3DDEMO_MAP) $(DSS_PCOUNT3DDEMO_OUT) $(DSS_PCOUNT3DDEMO_METAIMG_BIN) $(DSS_PCOUNT3DDEMO_DEPENDS) $(DSS_PCOUNT3DDEMO_ROV_XS)
	@echo 'Cleaning the Millimeter Wave Demo DSS RTSC package'
	@$(DEL) $(DSS_PCOUNT3DDEMO_CONFIGPKG)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(DSS_PCOUNT3DDEMO_DEPENDS)

