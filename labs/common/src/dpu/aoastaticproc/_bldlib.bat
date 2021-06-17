@REM ###############################################################################
@REM #
@REM # Set up tools and build environment variables for staticdetproc sdk
@REM #
@REM ###############################################################################

@REM ###############################################################################
@REM # Build variables (to be modified based on build need)
@REM ###############################################################################

set MMWAVE_SDK_TOOLS_PATH=c:/ti/mmwave_sdk_03_03_00_02
set MMWAVE_INDUSTRIAL_TOOL_PATH=c:/ti/mmwave_industrial_toolbox_4_0_0


cd  %MMWAVE_SDK_TOOLS_PATH%/packages/scripts/windows

call setenv.bat


cd  %MMWAVE_INDUSTRIAL_TOOL_PATH%/labs/common/src/dpu/aoastaticproc

gmake clean all



