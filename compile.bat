@echo off
REM Compile Chrono Heist Game

echo ================================================
echo Chrono Heist - Compilation
echo ================================================
echo.

echo Compiling game...
g++ -std=c++17 -O2 ChronoHeist.cpp -o ChronoHeist.exe -lfreeglut -lopengl32 -lglu32 -static-libstdc++ -static-libgcc

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ================================================
    echo Compilation Complete!
    echo ================================================
    echo.
    echo Game compiled successfully.
    echo.
    echo To run the game:
    echo   .\ChronoHeist.exe
    echo.
) else (
    echo.
    echo Compilation failed! Check for errors above.
    echo Make sure MinGW-w64 (or MSYS2) with freeglut/opengl libraries is installed and on PATH.
)

pause

