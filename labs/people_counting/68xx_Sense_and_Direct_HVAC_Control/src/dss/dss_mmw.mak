###################################################################################
#   Millimeter wave Demo
#
#  NOTE:
#      (C) Copyright 2016 Texas Instruments, Inc.
###################################################################################

###################################################################################
# Millimeter Wave Demo
###################################################################################
.PHONY: dssTMDemo dssTMDemoClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c dss
vpath %.c common
vpath %.c $(C64Px_DSPLIB_INSTALL_PATH)/packages
vpath %.c $(C674x_DSPLIB_INSTALL_PATH)/packages
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/lib/c674x-dsplib_1_03_00_01/src/DSPF_sp_fftSPxSP
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/modules/detection/CFAR/src
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/modules/DoA/CaponBF/src
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/modules/rangeProc/rangeProc/src
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/modules/utilities
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/chains/RadarReceiverPeopleCounting

ifeq ($(TRACKEROPT), NON3D)
TRACKLOCAATION = $(MMWAVE_SDK_INSTALL_PATH)/ti/alg/gtrack_ver3000
TRACKERLIBNAME = libgtrack_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)
TRACKOPT       = GTRACK_V1
else
TRACKLOCAATION = $(MMWAVE_SDK_INSTALL_PATH)/ti/alg/gtrack
TRACKERLIBNAME = libgtrack2D.$(R4F_LIB_EXT)
TRACKOPT       = GTRACK_2D
endif


###################################################################################
# Additional libraries which are required to build the DEMO:
###################################################################################
DSS_MMW_DEMO_STD_LIBS = $(C674_COMMON_STD_LIB)	      					\
   			-llibcrc_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)			\
   			-llibuart_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)			\
   			-llibmailbox_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)		\
   			-llibmmwavelink_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)		\
   			-llibmmwave_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)		\
   			-llibadcbuf_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)      	        \
   			-llibedma_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)   		\
                        -llibmmwavealg_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT) 		\
                        -ldsplib.ae64P							\
                        -ldsplib.ae674							

DSS_MMW_DEMO_LOC_LIBS = $(C674_COMMON_LOC_LIB)						\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/uart/lib		\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/crc/lib			\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/mailbox/lib	        \
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/adcbuf/lib	        \
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/edma/lib		\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwavelink/lib	        \
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwave/lib  		\
            		-i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/lib 		\
            		-i$(C674x_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/lib		\
           		-i$(MMWAVE_SDK_INSTALL_PATH)/ti/alg/mmwavelib/lib

###################################################################################
# Millimeter Wave Demo
###################################################################################
DSS_MMW_DEMO_CFG       = dss/dss_mmw.cfg
DSS_MMW_DEMO_CONFIGPKG = dss/mmw_configPkg_$(MMWAVE_SDK_DEVICE_TYPE)
DSS_MMW_DEMO_MAP       = $(MMWAVE_SDK_DEVICE_TYPE)_mmw_demo_dss.map
DSS_MMW_DEMO_OUT       = $(MMWAVE_SDK_DEVICE_TYPE)_mmw_demo_dss.$(C674_EXE_EXT)
DSS_MMW_DEMO_BIN       = $(MMWAVE_SDK_DEVICE_TYPE)_mmw_demo_dss.bin
DSS_MMW_DEMO_CMD       = dss/dss_mmw_linker.cmd
DSS_MMW_DEMO_SOURCES   = dss_main.c \
                         dss_config_edma_util.c \
                         dss_data_path.c \
            radarProcess.c \
            RADARDEMO_detectionCFAR.c \
            RADARDEMO_detectionCFAR_priv.c \
            RADARDEMO_rangeProc_utils.c \
			RADARDEMO_rangeProc.c \
			RADARDEMO_rangeProc_priv.c \
            cycle_measure.c \
            radarOsal_malloc.c \
			RADARDEMO_aoaEstCaponBF.c \
			RADARDEMO_aoaEstCaponBF_DopplerEst.c \
			RADARDEMO_aoaEstCaponBF_heatmapEst.c \
			RADARDEMO_aoaEstCaponBF_matrixInv.c

DSS_MMW_DEMO_DEPENDS   = $(addprefix $(PLATFORM_OBJDIR)/, $(DSS_MMW_DEMO_SOURCES:.c=.$(C674_DEP_EXT)))
DSS_MMW_DEMO_OBJECTS   = $(addprefix $(PLATFORM_OBJDIR)/, $(DSS_MMW_DEMO_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# RTSC Configuration:
###################################################################################
dssDemoRTSC: $(DSP_CFG)
	@echo 'Configuring RTSC packages...'
	$(XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(C674_XSFLAGS) -o $(DSS_MMW_DEMO_CONFIGPKG) $(DSS_MMW_DEMO_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

###################################################################################
# Build the Millimeter Wave Demo
###################################################################################
#C674_CFLAGS:=$(filter-out -ms0,$(C674_CFLAGS))
dssDemo: BUILD_CONFIGPKG=$(DSS_MMW_DEMO_CONFIGPKG)
dssDemo: C674_CFLAGS += --cmd_file=$(BUILD_CONFIGPKG)/compiler.opt -k -os -d$(TRACKOPT)\
                        -i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft16x16_imre/c64P	\
		        -i$(MMWAVE_DEMO_DEV_PATH)/lib/c674x-dsplib_1_03_00_01/src/DSPF_sp_fftSPxSP \
                        -i$(MMWAVE_DEMO_DEV_PATH)/radarDemo \
                        -i$(MMWAVE_DEMO_DEV_PATH)/radarDemo/common \
                        -i$(C64Px_DSPLIB_INSTALL_PATH)/packages \
                        -i$(C674x_DSPLIB_INSTALL_PATH)/packages \

dssDemo: buildDirectories dssDemoRTSC $(DSS_MMW_DEMO_OBJECTS)
	$(C674_LD) $(C674_LDFLAGS) $(DSS_MMW_DEMO_LOC_LIBS) $(DSS_MMW_DEMO_STD_LIBS) 			\
	-l$(DSS_MMW_DEMO_CONFIGPKG)/linker.cmd --map_file=$(DSS_MMW_DEMO_MAP) $(DSS_MMW_DEMO_OBJECTS) 	\
	$(PLATFORM_C674X_LINK_CMD) $(DSS_MMW_DEMO_CMD) $(C674_LD_RTS_FLAGS) -o $(DSS_MMW_DEMO_OUT)
	# @echo 'Built the DSS Millimeter Wave Demo [Preparing the BIN Format]'
	# @$(GENERATE_BIN) $(DSS_MMW_DEMO_OUT) $(DSS_MMW_DEMO_BIN)
	# @echo '******************************************************************************'
	# @echo 'Built the DSS Millimeter Wave OUT & BIN Formats'
	# @echo '******************************************************************************'

###################################################################################
# Cleanup the Millimeter Wave Demo
###################################################################################
dssDemoClean:
	@echo 'Cleaning the DSS Millimeter Wave Demo Objects'
	@rm -f $(DSS_MMW_DEMO_OBJECTS) $(DSS_MMW_DEMO_MAP) $(DSS_MMW_DEMO_OUT) $(DSS_MMW_DEMO_BIN) $(DSS_MMW_DEMO_DEPENDS)
	@echo 'Cleaning the DSS Millimeter Wave Demo RTSC package'
	@$(DEL) $(DSS_MMW_DEMO_CONFIGPKG)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(DSS_MMW_DEMO_DEPENDS)

