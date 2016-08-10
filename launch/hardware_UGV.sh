xterm -e roscore &
sleep 4
xterm -e roslaunch cooperative_obstacle_avoidance webcam.launch &
xterm -e rosrun image_view image_view image:=/image_raw &
xterm -e rosrun p2os_driver p2os_driver &


