


-- DEPRECATED -- 


see ros2cs_ws readme for updated process








-------------------------------------------------------------------------------------------------------------------------------------------------------------



In order to use nuget packages inside your ros2 C# nodes, you must obtain the .dll files from the nuget repository and install them using your cmakelists.txt

The first step for this is to go to the nuget repository and locate the package you want to install. https://www.nuget.org/
Search for the package you want to install. 

Then you must download the .nupkg file associated with the CORRECT VERSION of that library. I have found success with packages compatible with dotnet framework 3.5, but 
you can follow the succeding steps with any version and if it's the wrong one, the build will error letting you know; so feel free to try any version you like.

You may download a specific version of a nuget package by going to the "Versions" tab and clicking your desired version, then clicking Download Package on the right sidebar.

Once you have the .nupkg file, you will need to extract it (.7zip and Ubuntu File Explorer work fine)

Then open the extracted files, go inside the lib folder, then net[versionnumber] folder. Again, if you are unsure, follow the succeding steps and try a different one if it errors on build

inside that folder should be a few files including a .dll. This is the file you want. 

Copy that .dll file into your project.


Now for our additions to cmakelists.txt

The standard ros2cs cmakelists.txt has a line similar to:
set(_assemblies_dep_dlls
    ${ros2cs_common_ASSEMBLIES_DLL}
    ${ros2cs_core_ASSEMBLIES_DLL}
    ${std_msgs_ASSEMBLIES_DLL}
    )

inside this statement, you need to put the path to the .dll you just copied into your project. 

The line should look something like this: ${CMAKE_SOURCE_DIR}/nuget_packages/MathNet.Numerics.dll


ex:

set(_assemblies_dep_dlls
    ${ros2cs_common_ASSEMBLIES_DLL}
    ${ros2cs_core_ASSEMBLIES_DLL}
    ${std_msgs_ASSEMBLIES_DLL}
    ${CMAKE_SOURCE_DIR}/nuget_packages/MathNet.Numerics.dll
)


Once you have updates your cmakelists.txt you should be free to build the package and use the library.

If there is an error with versioning upon build or not being able to find the function you used, CHECK VERSIONING!