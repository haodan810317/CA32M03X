@echo off
cd /d "%~dp0"

for /d /r %%i in (Listings* Objects* JLinkLog.txt* JLinkSettings.ini*) do (
    echo Deleting "%%i"
    rd /s /q "%%i"
)

for /r %%i in (JLinkLog.txt* JLinkSettings.ini* *uvguix* *scvd*) do (
    echo Deleting "%%i"
    del "%%i"
)

echo All Listings folders have been deleted.
pause
