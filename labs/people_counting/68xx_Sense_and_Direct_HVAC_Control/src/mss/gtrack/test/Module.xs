/******************************************************************************
 * FILE PURPOSE: TEST Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION:
 *  This file contains the module specification for the package documentation.
 *
 * Copyright (C) 2016, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule (java.lang.System.getenv("MMWAVE_SDK_INSTALL_PATH") + "/scripts/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the Unit Test files to the release
 *  package
 **************************************************************************/
function modBuild()
{
    /* Add all the .c files to the release package. */
    var srcFiles = libUtility.listAllFiles (".c", "test/common");
    for (var k = 0 ; k < srcFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = srcFiles[k];
    
    /* Add all the .h files to the release package. */
    var incFiles = libUtility.listAllFiles (".h", "test/common");
    for (var k = 0 ; k < incFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = incFiles[k];

    /* Add necessary test vectors to the release package. */
    var miscFiles = libUtility.listAllFiles (".bin", "test/vectors/usecases/people_counting");
    for (var k = 0 ; k < miscFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = miscFiles[k];
    
    for (var device=0; device < DeviceType.length; device++)
    {
        /* Add all the test files for supported devices to the release package. */
        if ((DeviceType[device] == "xwr16xx") || (DeviceType[device] == "xwr18xx") || (DeviceType[device] == "xwr68xx"))
        {
            searchDir = "test/usecases"

            /* Add all the .c files to the release package. */
            var srcFiles = libUtility.listAllFiles (".c", searchDir);
            for (var k = 0 ; k < srcFiles.length; k++)
                Pkg.otherFiles[Pkg.otherFiles.length++] = srcFiles[k];

            /* Add all the .h files to the release package. */
            var incFiles = libUtility.listAllFiles (".h", searchDir);
            for (var k = 0 ; k < incFiles.length; k++)
                Pkg.otherFiles[Pkg.otherFiles.length++] = incFiles[k];

            /* Add all the .cfg files to the release package. */
            var cfgFiles = libUtility.listAllFiles (".cfg", searchDir);
            for (var k = 0 ; k < cfgFiles.length; k++)
                Pkg.otherFiles[Pkg.otherFiles.length++] = cfgFiles[k];

            /* Add all the .mak files to the release package. */
            var makFiles = libUtility.listAllFiles (".mak", searchDir);
            for (var k = 0 ; k < makFiles.length; k++)
                Pkg.otherFiles[Pkg.otherFiles.length++] = makFiles[k];

            /* Add all the .cmd files to the release package. */
            var makFiles = libUtility.listAllFiles (".cmd", searchDir);
            for (var k = 0 ; k < makFiles.length; k++)
                Pkg.otherFiles[Pkg.otherFiles.length++] = makFiles[k];

            /* Add all the .map files to the release package. */
            var mapFiles = libUtility.listAllFiles (".map", searchDir);
            for (var k = 0 ; k < mapFiles.length; k++)
                Pkg.otherFiles[Pkg.otherFiles.length++] = mapFiles[k];

            /* Add all the executables files (R4F) to the release package. */
            var outFiles = libUtility.listAllFiles (".xer4f", searchDir);
            for (var k = 0 ; k < outFiles.length; k++)
                Pkg.otherFiles[Pkg.otherFiles.length++] = outFiles[k];

            /* Add all the executables files (674) to the release package. */
            var outFiles = libUtility.listAllFiles (".xe674", searchDir);
            for (var k = 0 ; k < outFiles.length; k++)
                Pkg.otherFiles[Pkg.otherFiles.length++] = outFiles[k];
        }
    }
}

