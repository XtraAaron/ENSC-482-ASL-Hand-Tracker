@echo off
taskkill /F /FI "WINDOWTITLE eq Receiver*" >nul 2>&1
taskkill /F /IM aslReader.exe >nul 2>&1
g++ aslReader.cpp -o aslReader.exe -lws2_32
start "HandTracker" cmd /k "aslenv\Scripts\python.exe handtracker.py"
start "Receiver" cmd /k "aslReader.exe"
echo Running ASL Tracker