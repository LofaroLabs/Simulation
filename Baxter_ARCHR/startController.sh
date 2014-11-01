#!/bin/sh
echo "======Starting window for hubo-ach and loading dynamixels.======"
cd /home/archr/projects/archr_Code/Simulation/Baxter_ARCHR && gnome-terminal --tab --command "hubo-ach start" --tab -e "python sendACHData.py"
sleep 5 #this should be made so it eaits until executino of top line to run below commands
echo "======Starting window for servers. Run bash servergripper.sh first in one window======"
cd /home/archr/ros_ws && gnome-terminal --tab --command "bash baxter.sh" #--tab -e "bash baxter.sh"
echo "======Then run bash controllerBoth.sh and bash gripper.sh in two additional windows======"
cd /home/archr/ros_ws && gnome-terminal --tab --command "bash baxter.sh" --tab -e "bash baxter.sh"
