@echo off
setlocal

where python >nul 2>nul
if %errorlevel%==0 (
  python "%~dp0scripts\dev.py" %*
) else (
  py "%~dp0scripts\dev.py" %*
)

exit /b %errorlevel%
