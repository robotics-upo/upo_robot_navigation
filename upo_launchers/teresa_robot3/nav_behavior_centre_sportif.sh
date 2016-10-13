#!/bin/bash
roslaunch upo_launchers real_nav_behavior.launch robot:=teresa_robot3 scenario:=centre_sportif xtion_nav:=false &
echo $!
sleep 5

#People localisation
roslaunch uva_launchers combined_tracking.launch

