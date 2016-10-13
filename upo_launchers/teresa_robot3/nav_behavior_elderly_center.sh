#!/bin/bash
roslaunch upo_launchers real_nav_behavior.launch robot:=teresa_robot3 scenario:=elderly_center xtion_nav:=false &
echo $!
sleep 5

#People localisation
roslaunch uva_launchers combined_tracking.launch

