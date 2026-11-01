#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
export QT_OPENGL=desktop QSG_RENDER_LOOP=basic
exec dbus-run-session -- ign gazebo -g -v 4 --render-engine ogre2 --gui-config /workspace/so101/gui.config
