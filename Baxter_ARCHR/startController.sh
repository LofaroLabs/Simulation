#!/bin/sh
echo "======Starting window for servers. Run bash servergripper.sh first in one window======"
cd /home/archr/ros_ws && gnome-terminal --tab --command "bash baxter.sh" #--tab -e "bash baxter.sh"
echo "======Then run bash controllerBoth.sh and bash gripper.sh in two additional windows======"
cd /home/archr/ros_ws && gnome-terminal --tab --command "bash baxter.sh" --tab -e "bash baxter.sh"
