@echo off
taskkill /F /IM aslReader.exe >nul 2>&1
g++ aslReader.cpp -o aslReader.exe -lws2_32
start "HandTracker" cmd /k "aslenv\Scripts\python.exe handtracker.py"
start "Receiver" cmd /k "aslReader.exe"
echo Running ASL Tracker