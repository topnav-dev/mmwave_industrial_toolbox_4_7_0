###################################################################################
# mmWave Demo Level makefile
###################################################################################
include $(MMWAVE_SDK_INSTALL_PATH)/ti/common/mmwave_sdk.mak
include ./mss/mss_mmw.mak
include ./dss/dss_mmw.mak

MMW_DEMO_BIN       = $(MMWAVE_SDK_DEVICE)_mmw_demo.bin

ifeq ($(MMWAVE_SDK_DEVICE_TYPE), xwr68xx)
MMW_DEMO_BIN_ADDR       = 0x00000006
else
MMW_DEMO_BIN_ADDR       = 0x00000006
endif


###################################################################################
# Standard Targets which need to be implemented by each mmWave SDK module. This
# plugs into the release scripts.
###################################################################################
.PHONY: all clean mmwDemoClean mmwDemo

# Clean: This cleans all the objects
mmwDemoClean: mssDemoClean dssDemoClean

# This builds all the supported demos
mmwDemo: mssDemo dssDemo
	@$(GENERATE_METAIMAGE) $(MMW_DEMO_BIN) $(MMW_DEMO_BIN_ADDR) $(MSS_MMW_DEMO_OUT) $(RADARSS_IMAGE_BIN) $(DSS_MMW_DEMO_OUT)
	# @$(GENERATE_METAIMAGE) $(MMW_DEMO_BIN) 0x00000006 $(MSS_MMW_DEMO_OUT) $(XWR16XX_RADARSS_IMAGE_BIN) $(DSS_MMW_DEMO_OUT)
	@$(DEL) $(MSS_MMW_DEMO_BIN) $(DSS_MMW_DEMO_BIN)
	@echo '******************************************************************************'
	@echo 'Built the mmw Demo Flash Binary '
	@echo '******************************************************************************'

# Clean: This cleans all the objects
clean: mmwDemoClean

# This builds all the supported demos
all: mmwDemo


# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the DEMO '
	@echo 'mmwDemo         -> Builds the mmw Demo (both MSS and DSS binaries)'
	@echo 'mmwDemoClean    -> Cleans the mmw Demo (both MSS and DSS binaries)'
	@echo 'mssDemo         -> Builds the MSS binary for the mmw Demo'
	@echo 'mssDemoClean    -> Cleans the MSS binary for mmw Demo '
	@echo 'dssDemo         -> Builds the DSS binary for the mmw Demo'
	@echo 'dssDemoClean    -> Cleans the DSS binary for mmw Demo '
	@echo '****************************************************************************************'

