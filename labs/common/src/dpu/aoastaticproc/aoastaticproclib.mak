###################################################################################
# aoastaticproc Library Makefile
###################################################################################
.PHONY: aoastaticproclib aoastaticproclibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src

###################################################################################
# Library Source Files:
###################################################################################
AOASTATICPROC_DSP_LIB_SOURCES = aoaprocdsp.c 			

###################################################################################
# Library objects
#     Build for R4 and DSP
###################################################################################
AOASTATICPROC_DSP_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOASTATICPROC_DSP_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# Library Dependency:
###################################################################################
AOASTATICPROC_DSP_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOASTATICPROC_DSP_LIB_SOURCES:.c=.$(C674_DEP_EXT)))

###################################################################################
# Library Names:
###################################################################################
AOASTATICPROC_DSP_C674_DRV_LIB = lib/libaoaproc_dsp_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)

###################################################################################
# Library Build:
#     - Build the R4 & DSP Library
###################################################################################
aoastaticprocDSPLib: C674_CFLAGS += -i$(C674x_MATHLIB_INSTALL_PATH)/packages \
								-i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft32x32/c64P \
								-i$(MMWAVE_INDUSTRIAL_TOOL_PATH)/labs
								
aoastaticprocDSPLib: buildDirectories $(AOASTATICPROC_DSP_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(C674_AR) $(C674_AR_OPTS) $(AOASTATICPROC_DSP_C674_DRV_LIB) $(AOASTATICPROC_DSP_C674_DRV_LIB_OBJECTS)

aoastaticproclib: aoastaticprocDSPLib


###################################################################################
# Clean the Libraries
###################################################################################
aoastaticprocDSPLibClean:
	@echo 'Cleaning the aoastaticproc DSP Library Objects'
	@$(DEL) $(AOASTATICPROC_DSP_C674_DRV_LIB_OBJECTS) $(AOASTATICPROC_DSP_C674_DRV_LIB)
	@$(DEL) $(AOASTATICPROC_DSP_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)


aoastaticproclibClean: aoastaticprocDSPLibClean


###################################################################################
# Dependency handling
###################################################################################
-include $(AOASTATICPROC_DSP_C674_DRV_DEPENDS)

