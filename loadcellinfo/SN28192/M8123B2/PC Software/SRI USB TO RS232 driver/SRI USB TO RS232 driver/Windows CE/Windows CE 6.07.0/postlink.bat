REM (C)2010-13 David Jones
REM Thanks to Mike Hall

REM CEContentWiz
@echo off
@echo Executing CEContentWiz Postlink.bat
@echo http://CEContentWiz.codeplex.com
@echo .

if %_WINCEOSVER%WXYZ == WXYZ GOTO SKIP

@echo copying Content Files from Resource Files folder to Targeted Debug Directory
if %_WINCEOSVER%==600 (
copy "%BUILDROOT%\Resources\*.*" "%_PROJECTROOT%\cesysgen\oak\target\%_TGTCPU%\%WINCEDEBUG%" /Y
) else if %_WINCEOSVER%==700 (
copy "%BUILDROOT%\Resources\*.*" "%SG_OUTPUT_ROOT%\oak\target\%_TGTCPU%\%WINCEDEBUG%"  /Y
) else if %_WINCEOSVER%==800 (
REM Small change for Compact 2013
copy ".\Resources\*.*" "%SG_OUTPUT_ROOT%\oak\target\%_TGTCPU%\%WINCEDEBUG%"  /Y

) else (
echo Compact/CE Version:%_WINCEOSVER%  Not supported
GOTO SKIP
)

@echo .

@echo copying Content Files from Resource Files folder to FlatRelease Directory
rem copy "%BUILDROOT%\Resources\*.*" %_FLATRELEASEDIR%  /Y
if %_WINCEOSVER%==600 (
copy "%BUILDROOT%\Resources\*.*" %_FLATRELEASEDIR% /Y
) else if %_WINCEOSVER%==700 (
copy "%BUILDROOT%\Resources\*.*" %_FLATRELEASEDIR%  /Y
) else if %_WINCEOSVER%==800 (
REM Small change for Compact 2013
copy ".\Resources\*.*" %_FLATRELEASEDIR%  /Y

) else (
echo Compact/CE Version:%_WINCEOSVER%  Not supported
GOTO SKIP
)

@echo .
@echo Building .cab file
@echo .

PUSHD
cd %_FLATRELEASEDIR%
IF EXIST FTDI_VCP_x86_CE700.inf (
    cabwiz FTDI_VCP_x86_CE700.inf
	IF EXIST FTDI_VCP_x86_CE700.cab (
		@echo Generated .cab file: FTDI_VCP_x86_CE700.cab in FLATRELEASEDIR.
	) else (
		@echo Generation of .cab file: FTDI_VCP_x86_CE700.cab failed.
	)
)else (
	@echo No file FTDI_VCP_x86_CE700.inf for .cab file generation
)
 
POPD

@echo .
@echo Done Copying
@echo .

:SKIP

