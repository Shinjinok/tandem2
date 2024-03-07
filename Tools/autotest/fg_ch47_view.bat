set AUTOTESTDIR="%~dp0\aircraft"
c:
FOR /F "delims=" %%D in ('dir /b "\Program Files"\FlightGear*') DO set FGDIR=%%D
echo "Using FlightGear %FGDIR%"
cd "\Program Files\%FGDIR%\bin"

fgfs ^
    --generic=socket,in,30,127.0.0.1,9002,udp,MAVLink ^
    --generic=socket,out,30,127.0.0.1,9003,udp,MAVLink ^
    --model-hz=1000 ^
    --aircraft=ch47 ^
    --fg-aircraft=%AUTOTESTDIR% ^
    --airport=PHNL ^
    --geometry=650x550 ^
    --bpp=32 ^
    --disable-hud-3d ^
    --disable-horizon-effect ^
    --timeofday=noon ^
    --disable-sound ^
    --disable-fullscreen ^
    --disable-random-objects ^
    --disable-ai-models ^
    --fog-disable ^
    --disable-specular-highlight ^
    --disable-anti-alias-hud ^
    --wind=0@0
pause
