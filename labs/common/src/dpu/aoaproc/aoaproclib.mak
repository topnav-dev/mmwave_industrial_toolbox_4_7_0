###################################################################################
# aoaproc Library Makefile
###################################################################################
.PHONY: aoaprocLib aoaprocLibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src

###################################################################################
# Library Source Files:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
AOAPROC_HWA_LIB_SOURCES = aoaprochwa.c
endif
AOAPROC_DSP_LIB_SOURCES = aoaprocdsp.c 			
AOA2DPROC_DSP_LIB_SOURCES = aoa2dprocdsp.c 			

###################################################################################
# Library objects
#     Build for R4 and DSP
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
AOAPROC_HWA_R4F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROC_HWA_LIB_SOURCES:.c=.$(R4F_OBJ_EXT)))
AOAPROC_HWA_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROC_HWA_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))
endif

AOAPROC_DSP_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROC_DSP_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))
AOA2DPROC_DSP_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOA2DPROC_DSP_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# Library Dependency:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
AOAPROC_HWA_R4F_DRV_DEPENDS  = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROC_HWA_LIB_SOURCES:.c=.$(R4F_DEP_EXT)))
AOAPROC_HWA_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROC_HWA_LIB_SOURCES:.c=.$(C674_DEP_EXT)))
endif

AOAPROC_DSP_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROC_DSP_LIB_SOURCES:.c=.$(C674_DEP_EXT)))
AOA2DPROC_DSP_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOA2DPROC_DSP_LIB_SOURCES:.c=.$(C674_DEP_EXT)))

###################################################################################
# Library Names:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
AOAPROC_HWA_R4F_DRV_LIB  = lib/libaoaproc_hwa_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)
AOAPROC_HWA_C674_DRV_LIB = lib/libaoaproc_hwa_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
endif

AOAPROC_DSP_C674_DRV_LIB = lib/libaoaproc_dsp_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
AOA2DPROC_DSP_C674_DRV_LIB = lib/libaoa2dproc_dsp_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
ifeq ($(MMWAVE_SDK_LIB_BUILD_OPTION),ODS)
AOA2DPROC_DSP_C674_DRV_LIB = lib/libaoa2dproc_dsp_ods_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
else
AOA2DPROC_DSP_C674_DRV_LIB = lib/libaoa2dproc_dsp_aop_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
endif


###################################################################################
# Library Build:
#     - Build the R4 & DSP Library
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
aoaprocHWALib: C674_CFLAGS += -i$(MMWAVE_INDUSTRIAL_TOOL_PATH)/labs 

aoaprocHWALib: R4F_CFLAGS  += -i$(MMWAVE_INDUSTRIAL_TOOL_PATH)/labs 

aoaprocHWALib: buildDirectories $(AOAPROC_HWA_R4F_DRV_LIB_OBJECTS) $(AOAPROC_HWA_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(R4F_AR) $(R4F_AR_OPTS) $(AOAPROC_HWA_R4F_DRV_LIB) $(AOAPROC_HWA_R4F_DRV_LIB_OBJECTS)
	$(C674_AR) $(C674_AR_OPTS) $(AOAPROC_HWA_C674_DRV_LIB) $(AOAPROC_HWA_C674_DRV_LIB_OBJECTS)
	
	
endif

aoaprocDSPLib: C674_CFLAGS += -i$(C674x_MATHLIB_INSTALL_PATH)/packages \
							  -i$(MMWAVE_INDUSTRIAL_TOOL_PATH)/labs \
							  -i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft32x32/c64P
							  
aoaprocDSPLib: buildDirectories $(AOAPROC_DSP_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(C674_AR) $(C674_AR_OPTS) $(AOAPROC_DSP_C674_DRV_LIB) $(AOAPROC_DSP_C674_DRV_LIB_OBJECTS)

aoa2dprocDSPLib: C674_CFLAGS += -i$(C674x_MATHLIB_INSTALL_PATH)/packages \
								-i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft32x32/c64P \
								-i$(MMWAVE_INDUSTRIAL_TOOL_PATH)/labs \
								-DAOA_TYPE_2D 

ifeq ($(MMWAVE_SDK_LIB_BUILD_OPTION),ODS)						
aoa2dprocDSPLib: C674_CFLAGS += -DANT_TYPE_ODS
endif

ifeq ($(MMWAVE_SDK_LIB_BUILD_OPTION),AOP)						
aoa2dprocDSPLib: C674_CFLAGS += -DANT_TYPE_AOP
endif								
aoa2dprocDSPLib: buildDirectories $(AOA2DPROC_DSP_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(C674_AR) $(C674_AR_OPTS) $(AOA2DPROC_DSP_C674_DRV_LIB) $(AOA2DPROC_DSP_C674_DRV_LIB_OBJECTS)

ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
aoaprocLib: aoaprocHWALib aoaprocDSPLib aoa2dprocDSPLib
else
aoaprocLib: aoaprocDSPLib aoa2dprocDSPLib
endif

###################################################################################
# Clean the Libraries
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
aoaprocHWALibClean:
	@echo 'Cleaning the aoaproc Library Objects'
	@$(DEL) $(AOAPROC_HWA_R4F_DRV_LIB_OBJECTS) $(AOAPROC_HWA_R4F_DRV_LIB)
	@$(DEL) $(AOAPROC_HWA_C674_DRV_LIB_OBJECTS) $(AOAPROC_HWA_C674_DRV_LIB)
	@$(DEL) $(AOAPROC_HWA_R4F_DRV_DEPENDS) $(AOAPROC_HWA_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)
endif

aoaprocDSPLibClean:
	@echo 'Cleaning the aoaproc DSP Library Objects'
	@$(DEL) $(AOAPROC_DSP_C674_DRV_LIB_OBJECTS) $(AOAPROC_DSP_C674_DRV_LIB)
	@$(DEL) $(AOAPROC_DSP_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

aoa2dprocDSPLibClean:
	@echo 'Cleaning the aoaproc DSP Library Objects'
	@$(DEL) $(AOA2DPROC_DSP_C674_DRV_LIB_OBJECTS) $(AOA2DPROC_DSP_C674_DRV_LIB)
	@$(DEL) $(AOA2DPROC_DSP_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)


ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
aoaprocLibClean: aoaprocHWALibClean aoaprocDSPLibClean aoa2dprocDSPLibClean
else
aoaprocLibClean: aoaprocDSPLibClean aoa2dprocDSPLibClean
endif

###################################################################################
# Dependency handling
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
-include $(AOAPROC_HWA_R4F_DRV_DEPENDS)
-include $(AOAPROC_HWA_C674_DRV_DEPENDS)
endif
-include $(AOAPROC_DSP_C674_DRV_DEPENDS)
-include $(AOA2DPROC_DSP_C674_DRV_DEPENDS)


