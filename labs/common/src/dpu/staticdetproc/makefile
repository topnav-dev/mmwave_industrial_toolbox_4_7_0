##################################################################################
# staticdetproc makefile
##################################################################################
# The following needs to be before any makefile inclusion
MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MODULE_NAME := $(notdir $(patsubst %/,%,$(dir $(MAKEFILE_PATH))))

include $(MMWAVE_SDK_INSTALL_PATH)/ti/common/mmwave_sdk.mak

##################################################################################
# SOC Specific Test Targets
##################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr16xx xwr18xx xwr68xx), )
include ./staticdetproclib.mak


###################################################################################
# Standard Targets which need to be implemented by each mmWave SDK module. This
# plugs into the release scripts.
###################################################################################
.PHONY: all clean help

##################1################################################################
# Build/Clean the library 
##################################################################################

# This builds the lib
all: 
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=ISK staticdetprocDSPLib
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=ODS staticdetprocDSPLib
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=AOP staticdetprocDSPLib

# This cleans the lib
clean:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=ISK staticdetprocDSPLibClean
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=ODS staticdetprocDSPLibClean
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=AOP staticdetprocDSPLibClean
	
	


# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for staticdetproc'
	@echo 'all                 -> Build lib and all unit tests'
	@echo 'clean               -> Clean out all the objects'
	@echo 'lib                 -> Build the Driver only'
	@echo 'libClean            -> Clean the Driver Library only'
	@echo '****************************************************************************************'
else
# For some reason the .DEFAULT does not seem to work for lib
.PHONY: lib

lib .DEFAULT:
	@echo "Error: $(MODULE_NAME) is not supported on $(MMWAVE_SDK_DEVICE) !!!"
endif

