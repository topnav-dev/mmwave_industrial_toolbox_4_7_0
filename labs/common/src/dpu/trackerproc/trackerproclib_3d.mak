###################################################################################
# trackerproc Library Makefile
###################################################################################
.PHONY: trackerprocLib trackerprocLibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src

###################################################################################
# Library Source Files:
###################################################################################

TRACKERPROC_LIB_SOURCES = trackerproc_3d.c

###################################################################################
# Library objects
#     Build for R4 and DSP
###################################################################################

TRACKERPROC_R4F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(TRACKERPROC_LIB_SOURCES:.c=.$(R4F_OBJ_EXT)))
TRACKERPROC_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(TRACKERPROC_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# Library Dependency:
###################################################################################
TRACKERPROC_R4F_DRV_DEPENDS  = $(addprefix $(PLATFORM_OBJDIR)/, $(TRACKERPROC_LIB_SOURCES:.c=.$(R4F_DEP_EXT)))
TRACKERPROC_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(TRACKERPROC_LIB_SOURCES:.c=.$(C674_DEP_EXT)))

###################################################################################
# Library Names:
###################################################################################
TRACKERPROC_R4F_DRV_LIB  = lib/libtrackerproc_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)
TRACKERPROC_C674_DRV_LIB = lib/libtrackerproc_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)


R4F_CFLAGS += --define=GTRACK_3D -i$(MMWAVE_INDUSTRIAL_TOOL_PATH)
C674_CFLAGS += --define=GTRACK_3D -i$(MMWAVE_INDUSTRIAL_TOOL_PATH)

###################################################################################
# Library Build:
#     - Build the R4 & DSP Library
###################################################################################
trackerprocR4FLib: buildDirectories $(TRACKERPROC_R4F_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(R4F_AR) $(R4F_AR_OPTS) $(TRACKERPROC_R4F_DRV_LIB) $(TRACKERPROC_R4F_DRV_LIB_OBJECTS)

trackerprocDSPLib: buildDirectories $(TRACKERPROC_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(C674_AR) $(C674_AR_OPTS) $(TRACKERPROC_C674_DRV_LIB) $(TRACKERPROC_C674_DRV_LIB_OBJECTS)

trackerprocLib: trackerprocR4FLib trackerprocDSPLib

###################################################################################
# Clean the Libraries
###################################################################################
trackerprocR4FLibClean:
	@echo 'Cleaning the trackerprocR4F Library Objects'
	@$(DEL) $(TRACKERPROC_R4F_DRV_LIB_OBJECTS) $(TRACKERPROC_R4F_DRV_LIB)
	@$(DEL) $(TRACKERPROC_R4F_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

trackerprocDSPLibClean:
	@echo 'Cleaning the trackerprocDSP Library Objects'
	@$(DEL) $(TRACKERPROC_C674_DRV_LIB_OBJECTS) $(TRACKERPROC_C674_DRV_LIB)
	@$(DEL) $(TRACKERPROC_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

trackerprocLibClean: trackerprocR4FLibClean trackerprocDSPLibClean

###################################################################################
# Dependency handling
###################################################################################
-include $(TRACKERPROC_R4F_DRV_DEPENDS)
-include $(TRACKERPROC_C674_DRV_DEPENDS)

