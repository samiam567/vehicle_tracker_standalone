# Copyright 2023 Alec Pannunzio <apun189@gmail.com>
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 

cmake_minimum_required(VERSION 3.8)

project(multivehicle_awareness C)

find_package(ament_cmake REQUIRED)
find_package(ros2cs_common REQUIRED)
find_package(ros2cs_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(rosidl_generator_cs REQUIRED)
find_package(blackandgold_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(autoware_auto_geometry_msgs REQUIRED)
find_package(autoware_auto_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(deep_orange_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_dotnet REQUIRED)

set(NUGET_PKGS_BASE ${PROJECT_BINARY_DIR}/../../../external/nuget_packages)

set(CSBUILD_TOOL "DotNetCore")
set(CSHARP_TARGET_FRAMEWORK "net6.0")
set(CS_ASM_DEP_DLLS_EXTRA
    #${NUGET_PKGS_BASE}/System.Threading.dll
    ${NUGET_PKGS_BASE}/System.Security.Permissions.dll
)

find_package(DotNETExtra REQUIRED)

set(_assemblies_dep_dlls
    ${ros2cs_common_ASSEMBLIES_DLL}
    ${ros2cs_core_ASSEMBLIES_DLL}
    ${tf2_dotnet_ASSEMBLIES_DLL}
    ${std_msgs_ASSEMBLIES_DLL}
    ${sensor_msgs_ASSEMBLIES_DLL}
    ${builtin_interfaces_ASSEMBLIES_DLL}
    ${blackandgold_msgs_ASSEMBLIES_DLL}
    ${autoware_auto_perception_msgs_ASSEMBLIES_DLL}
    ${autoware_auto_geometry_msgs_ASSEMBLIES_DLL}
    ${autoware_auto_geometry_ASSEMBLIES_DLL}
    ${geometry_msgs_ASSEMBLIES_DLL}
    ${visualization_msgs_ASSEMBLIES_DLL}
    ${nav_msgs_ASSEMBLIES_DLL}
    ${deep_orange_msgs_ASSEMBLIES_DLL}
    # NuGet packages
    ${NUGET_PKGS_BASE}/MathNet.Numerics.dll
    ${NUGET_PKGS_BASE}/DynamicObj.dll
    ${NUGET_PKGS_BASE}/FSharp.Core.dll
    ${NUGET_PKGS_BASE}/Plotly.NET.dll
    ${NUGET_PKGS_BASE}/Newtonsoft.Json.dll
    # Other system-specific tweaks
    ${CS_ASM_DEP_DLLS_EXTRA}
)

include(CPack)


add_dotnet_executable(vehicle_tracker
  Vehicle_tracker_node.cs
  Vehicle_tracker.cs
  Vehicle.cs
  VehicleStates.cs
  PlanningUtils.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)

install_dotnet(vehicle_tracker DESTINATION lib/${PROJECT_NAME}/dotnet)
ament_package()
