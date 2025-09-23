# Impellor V3: Reverse-Engineering Tool for Fusion 360

Processes 3D scanned impeller meshes (.stl, .obj, .ply), aligns with ICP, segments blades, fits NURBS surfaces, exports STEP files, and automates CAD in Fusion 360. Built with vcpkg for dependencies and GitHub Actions for CI/CD.

## What's Included
- Fusion 360 Add-in (Python): UI for file selection and settings.
- CLI Engine (C++): Mesh processing, blade segmentation, surface fitting, STEP export (uses PCL/OCCT).
- CI/CD: GitHub Actions builds on Windows, installs vcpkg deps, runs tests, uploads artifacts.

## Features
- Imports .stl, .obj, .ply meshes (enhanced OBJ loader from main.cpp).
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
See INSTALL.docx for local setup and usage steps.

## Building with vcpkg and CI/CD
- **Local Build**: Use CMake with vcpkg toolchain (see CMakeLists.txt).
- **CI/CD**: Push to GitHub; Actions builds on Windows, installs deps via vcpkg, compiles `cli_engine.exe`, runs on sample mesh, uploads STEP/PLY artifacts.

## Version
- v3: Full v0.3 roadmap, plus ICP, blade segmentation, and vcpkg/GitHub Actions integration.

## License
MIT License (free to use/modify).

## Notes
Test with simple meshes (search "free impeller STL"). Provide multiple scans in config.json for ICP. Blade count must match geometry.
