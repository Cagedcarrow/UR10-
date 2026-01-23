@echo off

set batchFolder=%~dp0

cd /d %batchFolder%

set command=RegAsm.exe
set codeBase=/codebase
set dllName=SwRoboDKCmd.dll
set dllPath= "%batchFolder%%dllName%"

%command% %codeBase% %dllPath%

echo Done
echo ----------------------------------------------------------
echo IMPORTANT: 
echo If errors arised make sure you set to "Unblock" the RegAsm.exe file and the DLL files.
echo Also, make sure you run the BAT file as administrator.
pause
