@echo off
cd /d %~dp0
setlocal
for %%I in (python.exe) do if exist %%~$path:I set f=%%~$path:I
if exist %f% (
  %f:python.exe=%omniidl.exe -bpython -I"%RTM_ROOT%rtm\idl" -I"G:\RTC\CraneX7ControllerRTC-master\idl" idl/ManipulatorCommonInterface_Middle.idl idl/ManipulatorCommonInterface_Common.idl idl/ManipulatorCommonInterface_DataTypes.idl idl/BasicDataType.idl 
) else (
  echo "python.exe" can not be found.
  echo Please modify PATH environmental variable for python command.
)
endlocal
