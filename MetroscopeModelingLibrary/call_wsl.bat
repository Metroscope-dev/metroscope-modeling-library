@echo off

REM Wrapper for call_wsl.sh

set log=dslog.txt

REM fetch WSL path
call wslpath.bat
REM note it is important to not attempt to write to dslog.txt during normal operations since that will interfere with parallel execution.

if not exist "%WSLPath%" (
  goto wslCorrect
)

call "%WSLPath%" -e ./call_wsl.sh %*

goto done

:wslCorrect
echo WSL not found: "%WSLPath%" >> %log%
echo Please correct the WSL path. >> %log%
echo For instructions on how to install WSL, please visit http://www.dymola.com/compiler >> %log%

:done
