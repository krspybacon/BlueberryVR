@echo off
rem Builds and runs the synthetic SLAM test. Needs OpenCV from vcpkg (x64-windows-static) and a VS developer environment.
rem usage: build_slam_sim.bat [speed] [frames] [led 0/1] [imuNoiseDeg] [pixelNoise]
if "%VCPKG_INSTALLED%"=="" set VCPKG_INSTALLED=%USERPROFILE%\vcpkg\installed\x64-windows-static
set P=%~dp0..\src\samples\driver_HadesVR
if not exist "%~dp0out" mkdir "%~dp0out"
pushd "%~dp0out"
cl /nologo /std:c++17 /O2 /MT /EHsc /W1 /D_CRT_SECURE_NO_WARNINGS ^
 /I"%P%\include" /I"%VCPKG_INSTALLED%\include" /I"%VCPKG_INSTALLED%\include\opencv4" ^
 "%~dp0slam_sim.cpp" "%P%\src\SlamTracker.cpp" "%P%\src\driverlog.cpp" ^
 /Fe:slam_sim.exe /link /LIBPATH:"%VCPKG_INSTALLED%\lib" ^
 opencv_calib3d4.lib opencv_features2d4.lib opencv_imgproc4.lib opencv_imgcodecs4.lib opencv_videoio4.lib opencv_flann4.lib opencv_core4.lib ^
 jpeg.lib turbojpeg.lib spng_static.lib zs.lib ^
 kernel32.lib user32.lib gdi32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib comdlg32.lib shlwapi.lib ws2_32.lib ^
 mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib strmiids.lib setupapi.lib
if errorlevel 1 (popd & exit /b 1)
"%CD%\slam_sim.exe" %*
popd
