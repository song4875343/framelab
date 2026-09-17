@echo off
rem FrameLab Neutralino exe builder (keep this file ASCII-only).
rem Usage: run this bat inside the neutralino\ folder after editing frontend code.
rem Steps: 1) rebuild resources/ from project sources  2) neu update  3) neu build
rem Only this .bat and neutralino.config.json are tracked by git; everything
rem else under neutralino\ (resources/, bin/, dist/) is generated and ignored.
cd /d %~dp0

echo [1/3] rebuilding resources/ from project sources ...
if exist resources rmdir /s /q resources
mkdir resources\js
mkdir resources\icons
mkdir resources\example
copy /y ..\index.html resources\index.html
copy /y ..\solver.js resources\solver.js
copy /y ..\design.js resources\design.js
copy /y ..\xara.js resources\xara.js
copy /y ..\opensees_import.js resources\opensees_import.js
copy /y ..\example\*.json resources\example\
copy /y ..\example\*.py resources\example\
copy /y ..\example\*.tcl resources\example\

echo writing resources\js\app.js ...
powershell -NoProfile -Command "[IO.File]::WriteAllText('resources\js\app.js', '(function () { try { if (typeof Neutralino === \"undefined\") return; Neutralino.init(); Neutralino.events.on(\"windowClose\", function () { try { Neutralino.app.exit(); } catch (e) {} }); } catch (e) { if (window.console && console.warn) console.warn(\"[neutralino] init skipped:\", e); } })();')"
if errorlevel 1 (
  echo [ERROR] failed to write js\app.js.
  pause
  exit /b 1
)

echo generating resources\icons\appIcon.png ...
powershell -NoProfile -Command "Add-Type -AssemblyName System.Drawing; $b = New-Object System.Drawing.Bitmap(256, 256); $g = [System.Drawing.Graphics]::FromImage($b); $g.Clear([System.Drawing.Color]::FromArgb(10, 18, 36)); $br = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(110, 231, 255)); $f = New-Object System.Drawing.Font('Arial', 150, [System.Drawing.FontStyle]::Bold); $g.DrawString('F', $f, $br, 52, 28); $g.Dispose(); $f.Dispose(); $br.Dispose(); $b.Save('resources\icons\appIcon.png', [System.Drawing.Imaging.ImageFormat]::Png); $b.Dispose()"
if errorlevel 1 (
  echo [ERROR] failed to generate appIcon.png.
  pause
  exit /b 1
)

echo patching resources\index.html for Neutralino ...
rem NOTE: must use UTF-8 explicitly (PS 5.1 defaults to ANSI and corrupts Chinese).
powershell -NoProfile -Command "$t = Get-Content -LiteralPath 'resources\index.html' -Raw -Encoding utf8; if ($t -notmatch 'neutralino\.js') { $t = $t -replace '<script src=\"opensees_import\.js\"></script>', ('<script src=\"opensees_import.js\"></script>' + \"`r`n\" + '<script src=\"/js/neutralino.js\"></script>' + \"`r`n\" + '<script src=\"/js/app.js\"></script>') }; if ($t -notmatch 'rel=\"icon\"') { $t = $t -replace '<meta charset=\"UTF-8\" />', ('<meta charset=\"UTF-8\" />' + \"`r`n\" + '<link rel=\"icon\" type=\"image/png\" href=\"/icons/appIcon.png\" />') }; [IO.File]::WriteAllText('resources\index.html', $t)"

where neu >nul 2>nul
if errorlevel 1 (
  echo neu CLI not found, trying npx ...
  set NEU=npx --yes @neutralinojs/neu
) else (
  set NEU=neu
)

echo [2/3] fetching neutralino binaries and client ...
call %NEU% update
if errorlevel 1 (
  echo [ERROR] neu update failed. Check network.
  pause
  exit /b 1
)

echo [3/3] building exe ...
call %NEU% build
if errorlevel 1 (
  echo [ERROR] neu build failed.
  pause
  exit /b 1
)

echo.
echo Build done. Check dist\ folder:
dir dist
pause
