#how to run this crap?

	
	$ kirix@automn:~/.ros/camera_info$ ls
		head_camera.yaml
	
	$ roscore
	$ roslaunch usb_cam usb_cam-test.launch
	$ ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
	$ rosrun cvision talker.py

	$ rostopic pub /orientation cvision/Orientation "length: 485.0"
	$ rostopic echo /list_objects -c
	$ rosrun image_view image_view image:=/see_main

