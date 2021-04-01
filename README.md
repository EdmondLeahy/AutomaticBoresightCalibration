# AutomaticBoresightCalibration
Automatic Lidar Boresight calibration algorithm (Based on IfTheMapFits)

Follow this in order to use with Eclipse IDE: https://stackoverflow.com/questions/9453851/how-to-configure-eclipse-cdt-for-cmake

To set up a new project with existing source code, please follow these steps:

    1. Check out your source code.
    2. Open the new C/C++ project wizard ("File" => "New" => "C Project" or "File" => "New" => "C++ Project").
        Make sure the project location points to root directory of your checked out files
        For the project type, select Executable. You may also select Shared Library or Static Library, it does not matter, that information comes from your CMakeLists.txt, but CDT requires it. Do not select Makefile project here!
        Finish project creation.
    3. Open the "Project Properties" dialog.
        Select the "C/C++ Build" node and the "Builder Settings" tab and make sure Generate Makefiles automatically is checked.
        Select the "Tool Chain Editor" node and set "CMake Make Builder" as the current builder.
        If your top level CMakeLists.txt does not reside in the root directory of your checked out files, select the "C/C++ General" "Path and Symbols" node and the "Source Location" tab. Then adjust the source folder to point to the directory containing your CMakeLists.txt file. This will tell the CDT indexer to scan the header files in your project.
    Pro Tip: Add a CMakeLists.txt file in the root directory of your checked out files, if you miss the Binaries or Archives folder in the C/C++ Projects View. Build the project. This will invoke cmake to generate the build scripts, if necessary, and then will invoke make.

Open the "Project Properties" dialog.

    Select the "C/C++ General" node and the "Preprocessor Includes Paths, Macros etc." tab. Select "CMAKE_EXPORT_COMPILE_COMMANDS Parser" and move it to the top of the list.
    Hit "OK" to close the dialog. Make sure to trigger one build now and recreate the index.
