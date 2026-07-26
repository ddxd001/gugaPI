@echo off
cd /d "%~dp0"
start "" powershell -ExecutionPolicy Bypass -File server.ps1
timeout /t 1 >nul
start "" http://localhost:8080/
