#!/bin/sh
echo "======Starting window for servers. Run bash serverl1.sh, bash serverl2.sh======"
cd /home/archr/ros_ws && gnome-terminal --tab --command "bash baxter.sh" --tab -e "bash baxter.sh"
echo "======bash controllerl1.sh and bash controllerl2.sh separately here.======"
cd /home/archr/ros_ws && gnome-terminal --tab --command "bash baxter.sh" --tab -e "bash baxter.sh"
