cmake_minimum_required(VERSION 3.10)
project(ImpellorV1)

set(CMAKE_CXX_STANDARD 17)

# Note: Root CMakeLists.txt handles vcpkg dependencies
add_executable(cli_engine fit.cpp)
target_link_libraries(cli_engine ${PCL_LIBRARIES} ${OpenCASCADE_LIBRARIES} nlohmann_json::nlohmann_json)
target_include_directories(cli_engine PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCASCADE_INCLUDE_DIR})
target_compile_definitions(cli_engine PRIVATE ${PCL_DEFINITIONS})
