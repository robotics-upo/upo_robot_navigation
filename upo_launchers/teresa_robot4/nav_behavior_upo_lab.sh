#!/bin/bash
roslaunch upo_launchers real_nav_behavior.launch robot:=teresa_robot4 scenario:=upo_lab xtion_nav:=true &
echo $!
sleep 5

#People localisation
roslaunch uva_launchers combined_tracking.launch

