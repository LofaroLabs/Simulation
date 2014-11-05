#!/bin/sh
echo "======Starting window for hubo-ach and loading dynamixels.======"
cd /home/archr/projects/archr_Code/Simulation/Baxter_ARCHR && gnome-terminal --tab --command "hubo-ach start" --tab -e "sudo python sendACHData.py"
sleep 5 #this should be so it runs dependent on the above being successful
echo "======Starting window for servers. Run bash servergripper.sh first in one window======"
cd /home/archr/ros_ws && gnome-terminal --tab --command "bash baxter.sh" #--tab -e "bash baxter.sh"
echo "======Then run bash controllerBoth.sh and bash gripper.sh in two additional windows======"
cd /home/archr/ros_ws && gnome-terminal --tab --command "bash baxter.sh" --tab -e "bash baxter.sh"
