# Impellor V3: Reverse-Engineering Tool for Fusion 360

Processes 3D scanned impeller meshes (.stl, .obj, .ply), aligns with ICP, segments blades, fits NURBS surfaces, exports STEP files, and automates CAD in Fusion 360. Built with vcpkg for dependencies and GitHub Actions for CI/CD.

## What's Included

- **Fusion 360 Add-in (Python)**: UI for file selection and settings.
- **CLI Engine (C++)**: Mesh processing, blade segmentation, surface fitting, STEP export (uses PCL/OCCT).
- **CI/CD**: GitHub Actions builds on Windows, installs vcpkg dependencies, runs tests, uploads artifacts.

## Features

- Imports .stl, .obj, .ply meshes (enhanced OBJ loader).
- Aligns multiple partial scans using ICP (RMS ≤0.01 mm).
- Segments individual blades using Euclidean clustering and region growing.
- Fits bore cylinder, backplate/shroud planes using RANSAC.
- Extracts blade curve network (8–16 sections per blade).
- Fits NURBS surfaces to blades (RMS ≤0.02 mm).
- Auto-repairs gaps during sewing.
- Infers fillets at hub/shroud junctions.
- Generates deviation map (CSV, colorized PLY).
- Reports blade angles, thickness, camber line (JSON/CSV).
- Fusion automation: Imports STEP, stitches, patterns, fillets.

## How to Use

See `INSTALL.docx` for local setup and usage steps.

### Local Build

1. Clone the repository:
   ```bash
   git clone https://github.com/omorellAUS/Impellor-V3.git
   cd Impellor-V3


Set up vcpkg:git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg.exe install pcl[vtk,visualization,surface]:x64-windows-static opencascade:x64-windows-static nlohmann-json:x64-windows-static


Configure and build:cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows-static -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release


Run the binary:mkdir output
.\build\Release\fit.exe config.json



CI/CD

Push to GitHub; the workflow (.github/workflows/build.yml) builds on Windows, installs dependencies via vcpkg, compiles fit.exe, runs on test.obj, and uploads fit.exe and fit.pcd as artifacts.
Fixed vcpkg shallow clone issue by adding git fetch --unshallow in the workflow.

Version

v3: Full v0.3 roadmap, plus ICP, blade segmentation, and vcpkg/GitHub Actions integration.

License
MIT License (free to use/modify).
Notes

Test with simple meshes (search "free impeller STL" online).
Provide multiple scans in config.json for ICP.
Blade count in config.json must match the input mesh geometry.
Uses x64-windows-static triplet for static linking to avoid runtime dependency issues.

About
Reverse-engineering tool for impeller meshes, integrating with Fusion 360 for automated CAD generation.```
