@REM ###############################################################################
@REM #
@REM # Set up tools and build environment variables for aoaproc Industrial Tool Box
@REM #
@REM ###############################################################################

@REM ###############################################################################
@REM # Build variables (to be modified based on build need)
@REM ###############################################################################
echo off

set MMWAVE_SDK_TOOLS_PATH=c:/ti/mmwave_sdk_03_05_00_04

set CWD=%~dp0

for %%I in ("%CWD%..\..\..\..") do set "MMWAVE_INDUSTRIAL_TOOL_PATH=%%~fI"

set MMWAVE_INDUSTRIAL_TOOL_PATH=%MMWAVE_INDUSTRIAL_TOOL_PATH:\=/%

echo %MMWAVE_INDUSTRIAL_TOOL_PATH%

cd  %MMWAVE_SDK_TOOLS_PATH%/packages/scripts/windows

call setenv.bat

cd  %CWD%

gmake clean all



