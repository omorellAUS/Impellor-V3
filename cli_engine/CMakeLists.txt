cmake_minimum_required(VERSION 3.20)
project(ImpellorV3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Release)

# Set vcpkg toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "vcpkg toolchain file")

# Find PCL with specific components
find_package(PCL REQUIRED COMPONENTS core visualization surface)
if (NOT PCL_FOUND)
  message(FATAL_ERROR "PCL not found. Ensure vcpkg is configured correctly.")
endif()

# Find OpenCascade
find_package(OpenCASCADE REQUIRED)
if (NOT OpenCASCADE_FOUND)
  message(FATAL_ERROR "OpenCascade not found. Ensure vcpkg is configured correctly.")
endif()

# Find nlohmann-json
find_package(nlohmann_json CONFIG REQUIRED)

# Add executable
add_executable(fit cli_engine/fit.cpp)

# Include directories
target_include_directories(fit PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCASCADE_INCLUDE_DIR})

# Link libraries
target_link_libraries(fit PRIVATE ${PCL_LIBRARIES} ${OpenCASCADE_LIBRARIES} nlohmann_json::nlohmann_json)

# Add definitions for PCL
add_definitions(${PCL_DEFINITIONS})
