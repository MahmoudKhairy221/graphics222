@echo off
REM Compile Chrono Heist Game

echo ================================================
echo Chrono Heist - Compilation
echo ================================================
echo.

echo Compiling game...
g++ -o ChronoHeist.exe ChronoHeist.cpp -lGL -lGLU -lglut -lm

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ================================================
    echo Compilation Complete!
    echo ================================================
    echo.
    echo Game compiled successfully.
    echo.
    echo To run the game:
    echo   ChronoHeist.exe
    echo.
) else (
    echo.
    echo Compilation failed! Check for errors above.
    echo Make sure you have GLUT and OpenGL libraries installed.
)

pause

