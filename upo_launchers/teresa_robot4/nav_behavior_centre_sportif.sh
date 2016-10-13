#!/bin/bash
roslaunch upo_launchers real_nav_behavior.launch robot:=teresa_robot4 scenario:=centre_sportif xtion_nav:=true &
echo $!
sleep 5

#People localisation
roslaunch uva_launchers combined_tracking.launch

