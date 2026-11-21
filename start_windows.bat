@echo off
cd /d "%~dp0"
echo Setting up Python packages (first run can take a minute)...
python -m pip install --quiet -r requirements.txt
echo Starting the robotaxi server... (close this window to stop)
python app.py
pause
