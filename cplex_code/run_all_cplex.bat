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

echo [1/9] S1
%OPLRUN% %MOD% S1.dat > "%OUTDIR%\S1_output.txt" 2>&1

echo [2/9] S2
%OPLRUN% %MOD% S2.dat > "%OUTDIR%\S2_output.txt" 2>&1

echo [3/9] S3
%OPLRUN% %MOD% S3.dat > "%OUTDIR%\S3_output.txt" 2>&1

echo [4/9] M1
%OPLRUN% %MOD% M1.dat > "%OUTDIR%\M1_output.txt" 2>&1

echo [5/9] M2
%OPLRUN% %MOD% M2.dat > "%OUTDIR%\M2_output.txt" 2>&1

echo [6/9] M3
%OPLRUN% %MOD% M3.dat > "%OUTDIR%\M3_output.txt" 2>&1

echo [7/9] L1
%OPLRUN% %MOD% L1.dat > "%OUTDIR%\L1_output.txt" 2>&1

echo [8/9] L2
%OPLRUN% %MOD% L2.dat > "%OUTDIR%\L2_output.txt" 2>&1

echo [9/9] L3
%OPLRUN% %MOD% L3.dat > "%OUTDIR%\L3_output.txt" 2>&1

echo.
echo Bitti!
pause