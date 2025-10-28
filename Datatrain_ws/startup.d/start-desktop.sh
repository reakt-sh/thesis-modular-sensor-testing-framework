#!/usr/bin/env bash
set -e
export USER=dev
export DISPLAY=${DISPLAY:-:1}
export VNC_PORT=${VNC_PORT:-5901}
export VNC_RESOLUTION=${VNC_RESOLUTION:-1680x1050}
export VNC_COL_DEPTH=${VNC_COL_DEPTH:-24}

# Start the VNC server with XFCE
su - dev -c "printf '%s\n' \
'#!/usr/bin/env bash' \
'xrdb $HOME/.Xresources' \
'startxfce4 &' \
> ~/.vnc/xstartup && chmod +x ~/.vnc/xstartup"

# Kill any stale server
su - dev -c "vncserver -kill ${DISPLAY} >/dev/null 2>&1 || true"
# Start fresh
su - dev -c "vncserver ${DISPLAY} -geometry ${VNC_RESOLUTION} -depth ${VNC_COL_DEPTH}"

# Keep the process in foreground to let supervisor manage restart (tail log)
tail -f /home/dev/.vnc/*${DISPLAY}.log