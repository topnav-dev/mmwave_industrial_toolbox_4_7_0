###################################################################################
# staticdetproc Library Makefile
###################################################################################
.PHONY: staticdetprocDSPLib staticdetprocDSPLibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src

###################################################################################
# Library Source Files:
###################################################################################
STATICDETPROC_DSP_LIB_SOURCES = staticdetprocdsp.c 			

###################################################################################
# Library objects
###################################################################################
STATICDETPROC_DSP_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(STATICDETPROC_DSP_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# Library Dependency:
###################################################################################

STATICDETPROC_DSP_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(STATICDETPROC_DSP_LIB_SOURCES:.c=.$(C674_DEP_EXT)))

###################################################################################
# Library Names:
###################################################################################
ifeq ($(MMWAVE_SDK_LIB_BUILD_OPTION),ODS)
STATICDETPROC_DSP_C674_DRV_LIB = lib/libstaticdetproc_dsp_ods_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
else
ifeq ($(MMWAVE_SDK_LIB_BUILD_OPTION),AOP)
STATICDETPROC_DSP_C674_DRV_LIB = lib/libstaticdetproc_dsp_aop_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
else
STATICDETPROC_DSP_C674_DRV_LIB = lib/libstaticdetproc_dsp_isk_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
endif
endif

###################################################################################
# Library Build:
#     - Build the R4 & DSP Library
###################################################################################

staticdetprocDSPLib: C674_CFLAGS += -i$(C674x_MATHLIB_INSTALL_PATH)/packages \
								-i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft32x32/c64P \
								-i$(MMWAVE_INDUSTRIAL_TOOL_PATH)/ 

ifeq ($(MMWAVE_SDK_LIB_BUILD_OPTION),ODS)						
staticdetprocDSPLib: C674_CFLAGS += -DXWR68XX_ODS_ANTENNA_PATTERN
endif

ifeq ($(MMWAVE_SDK_LIB_BUILD_OPTION),AOP)						
staticdetprocDSPLib: C674_CFLAGS += -DXWR68XX_AOP_ANTENNA_PATTERN
endif

ifeq ($(MMWAVE_SDK_LIB_BUILD_OPTION),ISK)		
staticdetprocDSPLib: C674_CFLAGS += -DXWR68XX_ISK_ANTENNA_PATTERN
endif
								
				
staticdetprocDSPLib: buildDirectories $(STATICDETPROC_DSP_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(C674_AR) $(C674_AR_OPTS) $(STATICDETPROC_DSP_C674_DRV_LIB) $(STATICDETPROC_DSP_C674_DRV_LIB_OBJECTS)
	
	
###################################################################################
# Clean the Libraries
###################################################################################
staticdetprocDSPLibClean:
	@echo 'Cleaning the staticdetproc DSP Library Objects'
	@$(DEL) $(STATICDETPROC_DSP_C674_DRV_LIB_OBJECTS) $(STATICDETPROC_DSP_C674_DRV_LIB)
	@$(DEL) $(STATICDETPROC_DSP_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)
	

###################################################################################
# Dependency handling
###################################################################################
-include $(STATICDETPROC_DSP_C674_DRV_DEPENDS)

