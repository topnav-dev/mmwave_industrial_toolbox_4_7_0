###################################################################################
#   Millimeter wave Demo
#
#  NOTE:
#      (C) Copyright 2016 Texas Instruments, Inc.
###################################################################################

###################################################################################
# Millimeter Wave Demo
###################################################################################
.PHONY: mssDemo mssDemoClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c mss
vpath %.c common
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/modules/utilities
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/modules/classification/classificationUtilities/src/
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/modules/classification/featureExtraction/src/
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/modules/classification/classifier_kNN/src/
vpath %.c $(MMWAVE_DEMO_DEV_PATH)/radarDemo/modules/classification/classifier_kNN/utils/


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
MSS_MMW_DEMO_STD_LIBS = $(R4F_COMMON_STD_LIB)							\
		   	-llibpinmux_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT) 		\
		   	-llibcrc_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
		   	-llibdma_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
		   	-llibuart_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
		   	-llibmailbox_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
		   	-llibmmwavelink_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)	\
		   	-llibmmwave_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)        \
			-llibcli_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
			-l$(TRACKERLIBNAME)
			
MSS_MMW_DEMO_LOC_LIBS = $(R4F_COMMON_LOC_LIB)							\
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/pinmux/lib 	   		\
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/dma/lib				\
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/uart/lib			\
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/crc/lib				\
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/mailbox/lib	        \
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwavelink/lib	    \
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwave/lib          \
            -i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/cli/lib				\
            -i$(TRACKLOCAATION)/lib

###################################################################################
# Millimeter Wave Demo
###################################################################################
MSS_MMW_DEMO_CFG       = mss/mss_mmw.cfg
MSS_MMW_DEMO_CONFIGPKG = mss/mmw_configPkg_$(MMWAVE_SDK_DEVICE_TYPE)
MSS_MMW_DEMO_MAP       = $(MMWAVE_SDK_DEVICE_TYPE)_mmw_demo_mss.map
MSS_MMW_DEMO_OUT       = $(MMWAVE_SDK_DEVICE_TYPE)_mmw_demo_mss.$(R4F_EXE_EXT)
MSS_MMW_DEMO_BIN       = $(MMWAVE_SDK_DEVICE_TYPE)_mmw_demo_mss.bin
MSS_MMW_DEMO_CMD       = mss/mss_mmw_linker.cmd
MSS_MMW_DEMO_SOURCES   = mss_main.c \
						 task_mbox.c \
						 task_app.c \
						 gtrackAlloc.c \
						 gtrackLog.c \
						 radarOsal_malloc.c \
						 RADARDEMO_distanceUtil_priv.c \
						 RADARDEMO_featExtrUtil_priv.c \
						 RADARDEMO_svmKernelUtil_priv.c \
						 RADARDEMO_classifierkNN.c \
       					 RADARDEMO_classifierSVM.c \
						 RADARDEMO_classifierkNNCB.c \
						 RADARDEMO_featExtract.c \
						 classifierkNN_process.c \
						 Util_ClassifierkNN_inputParser.c \
						 Util_ClassifierkNN_targetManager.c \
                         cli.c
MSS_MMW_DEMO_DEPENDS   = $(addprefix  $(PLATFORM_OBJDIR)/, $(TRACKLOCAATION)/, $(MSS_MMW_DEMO_SOURCES:.c=.$(R4F_DEP_EXT)))
MSS_MMW_DEMO_OBJECTS   = $(addprefix $(PLATFORM_OBJDIR)/, $(MSS_MMW_DEMO_SOURCES:.c=.$(R4F_OBJ_EXT)))

###################################################################################
# RTSC Configuration:
###################################################################################
mssDemoRTSC:
	@echo 'Configuring RTSC packages...'
	$(XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(R4F_XSFLAGS) -o $(MSS_MMW_DEMO_CONFIGPKG) $(MSS_MMW_DEMO_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

###################################################################################
# Build the Millimeter Wave Demo
###################################################################################
mssDemo: BUILD_CONFIGPKG=$(MSS_MMW_DEMO_CONFIGPKG)
mssDemo: R4F_CFLAGS += --cmd_file=$(BUILD_CONFIGPKG)/compiler.opt --c99 -d$(TRACKOPT) -d_LITTLE_ENDIAN -i$(MMWAVE_DEMO_DEV_PATH)/radarDemo -i$(MMWAVE_DEMO_DEV_PATH)/api/notarget 
mssDemo: buildDirectories mssDemoRTSC $(MSS_MMW_DEMO_OBJECTS)
	$(R4F_LD) $(R4F_LDFLAGS) $(MSS_MMW_DEMO_LOC_LIBS) $(MSS_MMW_DEMO_STD_LIBS) 					\
	-l$(MSS_MMW_DEMO_CONFIGPKG)/linker.cmd --map_file=$(MSS_MMW_DEMO_MAP) $(MSS_MMW_DEMO_OBJECTS) 	\
	$(PLATFORM_R4F_LINK_CMD) $(MSS_MMW_DEMO_CMD) $(R4F_LD_RTS_FLAGS) -o $(MSS_MMW_DEMO_OUT)
	# @echo 'Built the MSS Millimeter Wave Demo [Preparing the BIN Format]'
	# @$(GENERATE_METAIMAGE) $(MSS_MMW_DEMO_OUT) $(MSS_MMW_DEMO_BIN)
	# @echo '******************************************************************************'
	# @echo 'Built the MSS Millimeter Wave OUT & BIN Formats'
	# @echo '******************************************************************************'

###################################################################################
# Cleanup the Millimeter Wave Demo
###################################################################################
mssDemoClean:
	@echo 'Cleaning the MSS Millimeter Wave Demo Objects'
	@rm -f $(MSS_MMW_DEMO_OBJECTS) $(MSS_MMW_DEMO_MAP) $(MSS_MMW_DEMO_OUT) $(MSS_MMW_DEMO_BIN) $(MSS_MMW_DEMO_DEPENDS)
	@echo 'Cleaning the MSS Millimeter Wave Demo RTSC package'
	@$(DEL) $(MSS_MMW_DEMO_CONFIGPKG)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(MSS_MMW_DEMO_DEPENDS) 

