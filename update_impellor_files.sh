#!/bin/bash
cat > config.json << 'EOF'
{
  "input_path": "Impeller.stl",
  "output_path": "output/fit.pcd",
  "icp_max_iterations": 100,
  "icp_max_correspondence_distance": 0.01,
  "ransac_distance_threshold": 0.02,
  "blade_count_min": 8,
  "blade_count_max": 20
}
EOF
cat > .github/workflows/build.yml << 'EOF'
name: Build Impellor V3
on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]
jobs:
  build:
    runs-on: windows-latest
    steps:
    - name: Checkout repository
      uses: actions/checkout@v4
      with:
        fetch-depth: 0
    - name: Setup CMake
      shell: powershell
      run: |
        $cmakeVersion = "3.20.0"
        $cmakeUrl = "https://github.com/Kitware/CMake/releases/download/v${cmakeVersion}/cmake-${cmakeVersion}-windows-x86_64.zip"
        $installPath = "C:\cmake"
        New-Item -ItemType Directory -Force -Path $installPath | Out-Null
        Invoke-WebRequest -Uri $cmakeUrl -OutFile "cmake.zip" -UseBasicParsing
        Expand-Archive -Path "cmake.zip" -DestinationPath $installPath
        $cmakeBinPath = Join-Path $installPath "cmake-${cmakeVersion}-windows-x86_64\bin"
        echo "CMAKE_DIR=$cmakeBinPath" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
        echo "$cmakeBinPath" | Out-File -FilePath $env:GITHUB_PATH -Encoding utf8 -Append
      env:
        GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
    - name: Verify CMake installation
      shell: powershell
      run: |
        cmake --version
        if ($LASTEXITCODE -ne 0) { exit 1 }
    - name: Setup vcpkg
      shell: powershell
      run: |
        git clone https://github.com/microsoft/vcpkg.git
        cd vcpkg
        .\bootstrap-vcpkg.bat
        cd ..
    - name: Install dependencies with vcpkg
      shell: powershell
      run: |
        .\vcpkg\vcpkg.exe install --triplet x64-windows-static
      env:
        VCPKG_DEFAULT_TRIPLET: x64-windows-static
      continue-on-error: false
    - name: Debug vcpkg installation
      shell: powershell
      run: |
        dir vcpkg\installed\x64-windows-static
        .\vcpkg\vcpkg.exe list
    - name: Clean up .DS_Store files
      shell: powershell
      run: |
        Remove-Item -Path .\.DS_Store -Force -ErrorAction SilentlyContinue
        Remove-Item -Path *\*.DS_Store -Force -ErrorAction SilentlyContinue
        git rm -r --cached .DS_Store --quiet || true
        git rm -r --cached */.DS_Store --quiet || true
    - name: Verify config.json and input mesh
      shell: powershell
      run: |
        if (-Not (Test-Path "config.json")) {
          Write-Error "config.json not found"
          exit 1
        }
        if (-Not (Test-Path "Impeller.stl")) {
          Write-Error "Impeller.stl not found in repository root"
          exit 1
        }
        Get-Content -Path config.json
        dir *.stl
    - name: Configure CMake
      shell: powershell
      run: |
        cmake -B build -S . -CMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows-static -DCMAKE_BUILD_TYPE=Release
        if ($LASTEXITCODE -ne 0) { exit 1 }
    - name: Build
      shell: powershell
      run: |
        cmake --build build --config Release
        if ($LASTEXITCODE -ne 0) { exit 1 }
    - name: Run Tests
      shell: powershell
      run: |
        mkdir output -Force
        .\build\Release\fit.exe config.json
      continue-on-error: true
    - name: Upload Artifacts
      uses: actions/upload-artifact@v4
      with:
        name: build-artifacts
        path: |
          build/Release/fit.exe
          output/fit.pcd
        if-no-files-found: warn
EOF
rm -f .DS_Store */.DS_Store
echo ".DS_Store" >> .gitignore
git rm -r --cached .DS_Store 2>/dev/null || true
git rm -r --cached */.DS_Store 2>/dev/null || true
git add config.json .github/workflows/build.yml .gitignore
echo "Files updated. Open GitHub Desktop to commit and push changes."
echo "Suggested commit message: 'Update build.yml, config.json, and .gitignore for 8-20 blade support'"