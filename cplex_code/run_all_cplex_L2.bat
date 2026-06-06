@echo off
chcp 65001 >nul


REM oplrun yolu
set OPLRUN="C:\Program Files\IBM\ILOG\CPLEX_Studio1210\opl\bin\x64_win64\oplrun.exe"

REM model
set MOD=adim_2_v2.mod

REM output klasoru
set OUTDIR=cplex_results
if not exist "%OUTDIR%" mkdir "%OUTDIR%"

REM kontrol
if not exist %OPLRUN% (
    echo HATA: oplrun.exe bulunamadi
    echo Yol: %OPLRUN%
    pause
    exit /b 1
)

echo.
echo Basliyor...
echo.



echo [1/1] L2
%OPLRUN% %MOD% L2.dat > "%OUTDIR%\L2_output.txt" 2>&1



echo.
echo Bitti!
pause