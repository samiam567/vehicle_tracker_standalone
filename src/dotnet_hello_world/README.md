# This is a collection of packages to help you get started working with ROS in C#.


## Notes

When creating your own packages be careful to include everything properly in your CMakeLists.txt, especially when it comes to custom messages. 

Messages all have members starting with capital letters. So a message that had a .data field in python will be .Data in C#


## Common errors and their meanings:



### Cmake error non-existant target
CMake Error at /home/apun1/on-vehicle/install/share/dotnet_cmake_module/cmake/Modules/FindDotNETExtra.cmake:112 (get_target_property):
  get_target_property() called with non-existent target
  "overtaking_controller".
Call Stack (most recent call first):
  CMakeLists.txt:49 (install_dotnet)

#### solution:

Make sure these two sections are correct in your CMakeLists.txt:


add_dotnet_executable(overtaking_controller
  overtaking_controller.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)

install_dotnet(overtaking_controller DESTINATION lib/${PROJECT_NAME}/dotnet)








