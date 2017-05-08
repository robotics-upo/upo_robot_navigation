# upo_launchers
Package with some configuration files and the launch files to run simulations of the navigation macro-action system.

Different scenarios are available (centre_sportif, elderly_center, upo_45, upo_lab, uva_demo and uva_lab). To run a simulation of the whole macro-action system for navigation, just move to the folder of the desired scenario and run the launch sim_(scenario_name)_nav_behavior.launch.

Then, in other terminal, run a tester to launch the different macro-actions:
rosrun upo_decision_making upo_nav_behavior_tester.py

The navigation goals for each scenario are predefined, and can be consulted in the cfg folder.

If you are only interested in testing the navigation to a goal, the navigation system without the macro-actions system can be lauched. Run the launch  upo_navigation.launch in the folder elderly_center. Regular navigation goals sent through Rviz are accepted.


