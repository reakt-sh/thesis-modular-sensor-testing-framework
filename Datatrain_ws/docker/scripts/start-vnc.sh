#!/usr/bin/env bash
set -e
: "${VNC_GEOMETRY:=1600x900}"

# Ensure xstartup exists
if [ ! -x "$HOME/.vnc/xstartup" ]; then
  mkdir -p "$HOME/.vnc"
  cat > "$HOME/.vnc/xstartup" <<'SH'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -r "$HOME/.Xresources" ] && xrdb "$HOME/.Xresources"
exec dbus-launch --exit-with-session startxfce4
SH
  chmod +x "$HOME/.vnc/xstartup"
fi

vncserver -kill :1 >/dev/null 2>&1 || true
vncserver :1 -localhost no -geometry "$VNC_GEOMETRY"

if [ -x /usr/share/novnc/utils/novnc_proxy ]; then
  /usr/share/novnc/utils/novnc_proxy --vnc localhost:5901 --listen 0.0.0.0:6080 &
else
  websockify --web=/usr/share/novnc/ 6080 localhost:5901 &
fi

echo "Open noVNC: http://localhost:6080"