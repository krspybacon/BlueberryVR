@echo off
rem Opens a VS developer environment (adjust the path to your Visual Studio) and runs the synthetic SLAM test.
call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
call "%~dp0build_slam_sim.bat" %*
