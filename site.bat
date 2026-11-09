@echo off
rem Puts the sim online at https://biobuzz.owenrossing.me while this window is open.
rem Invite codes are in sim\site\invite_codes.txt, one per line: CODE name
if exist "C:\Program Files\Android\Android Studio\jbr\bin\java.exe" set "JAVA_HOME=C:\Program Files\Android\Android Studio\jbr"
cd /d "%~dp0"
start "BioBuzz tunnel" cloudflared tunnel --no-autoupdate --url http://127.0.0.1:8090 run biobuzz-sim
if exist ..\gradlew.bat (call ..\gradlew.bat -p . site --console=plain) else (call gradlew.bat site --console=plain)
pause
