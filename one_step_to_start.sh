{
	gnome-terminal --tab "Navigation_1_Start" -- bash -c "cd src/vtol_navigation;python3 vtol_navigation_1.py"
 
}&
 
sleep 2s
{
	gnome-terminal --tab "Navigation_2_Start" -- bash -c "cd src/vtol_navigation;python3 vtol_navigation_2.py"
}&

sleep 2s
{
	gnome-terminal --tab "ROS_launch" -- bash -c ". devel/setup.bash;roslaunch ship_detector ship_detectors.launch"
}&

 
