#!/bin/bash
# Run script for Chrono Heist Game
# Usage: wsl -d Ubuntu -e bash -c "cd ~ && cp /mnt/c/Users/mahmo/Desktop/Graphics\ test/run_wsl.sh ~/run_wsl.sh && chmod +x ~/run_wsl.sh && ~/run_wsl.sh"

# Copy latest source from Windows
cp /mnt/c/Users/mahmo/Desktop/Graphics\ test/ChronoHeist.cpp ~/ChronoHeist.cpp

# Compile
echo "Compiling Chrono Heist..."
g++ -o ChronoHeist ~/ChronoHeist.cpp -lGL -lGLU -lglut -lm

if [ $? -eq 0 ]; then
    echo "Compilation successful!"
    echo "Running game..."
    DISPLAY=:0 ./ChronoHeist
else
    echo "Compilation failed! Check for errors above."
    echo "Make sure you have GLUT and OpenGL libraries installed."
fi

