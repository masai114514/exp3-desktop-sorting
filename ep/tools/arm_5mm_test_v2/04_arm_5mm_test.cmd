@echo off
setlocal
cd /d "%~dp0"
chcp 65001 >nul
python -c "import sys; sys.exit(0 if (3,9) <= sys.version_info[:2] <= (3,12) else 1)" >nul 2>&1
if not errorlevel 1 (
    python -X utf8 arm_5mm_test.py --connect
    goto finished
)
for %%V in (3.11 3.12 3.10 3.9) do (
    py -%%V -c "import sys" >nul 2>&1
    if not errorlevel 1 (
        py -%%V -X utf8 arm_5mm_test.py --connect
        goto finished
    )
)
echo Python 3.9-3.12 not found. Open this folder in VSCode and use your Python 3.11 terminal.
:finished
echo.
echo Keep this window open to read or screenshot the result.
pause
endlocal
