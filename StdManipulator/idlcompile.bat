echo off
setlocal
for %%I in (python.exe) do if exist %%~$path:I set f=%%~$path:I
if exist %f% do (
  %f:python.exe=%omniidl.exe -bpython -I"%RTM_ROOT%rtm\idl" -I"/home/isao-hara/ros_ws/work/MySawyer/StdManipulator/idl" idl/ManipulatorCommonInterface_Middle.idl idl/ManipulatorCommonInterface_Common.idl 
) else do (
  echo "python.exe" can not be found.
  echo Please modify PATH environmental variable for python command.
)
endlocal
