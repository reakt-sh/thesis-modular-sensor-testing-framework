#!/usr/bin/env bash
set -e
# Prepare VNC password for user 'dev'
su - dev -c "mkdir -p ~/.vnc && echo ${VNC_PASSWORD:-datatrain} | vncpasswd -f > ~/.vnc/passwd && chmod 600 ~/.vnc/passwd"