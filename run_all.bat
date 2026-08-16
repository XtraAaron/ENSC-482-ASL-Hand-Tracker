@echo off
g++ aslReader.cpp -o aslReader.exe -lws2_32
start "HandTracker" cmd /k "aslenv\Scripts\python.exe handtracker.py"
start "Receiver" cmd /k "aslReader.exe"