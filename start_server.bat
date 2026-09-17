@echo off
rem FrameLab xara/OpenSees server launcher (keep this file ASCII-only).
rem Usage: start_server.bat [port]
rem Steps: 1) uv sync (locked deps)  2) run xara_server.py
cd /d %~dp0

set PORT=8007
if not "%~1"=="" set PORT=%~1

where uv >nul 2>nul
if errorlevel 1 (
  echo [ERROR] 'uv' not found on PATH. Install it first:
  echo         https://docs.astral.sh/uv/
  pause
  exit /b 1
)

echo [1/2] uv sync ...
call uv sync
if errorlevel 1 (
  echo [ERROR] uv sync failed. Check network, pyproject.toml and uv.lock.
  pause
  exit /b 1
)

echo [2/2] starting xara server at http://127.0.0.1:%PORT% ...
echo       health: http://127.0.0.1:%PORT%/api/health
echo       page  : http://127.0.0.1:%PORT%/index.html  (Ctrl+C stops server)
call uv run python xara_server.py --port %PORT%
pause
