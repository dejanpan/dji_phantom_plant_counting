----------
Calibration
----------
Fly with the drone (Use DJI Go app) and shoot around 50 pictures of a checkeboard from close (0.5m), far (5m) and in between. Also from different angles.
Copy images in one folder and use image_file_to_ros_topic.cpp tool to generate a ROS bag.
Use https://github.int.bosppaa.com/deepfield/phenotyping/tree/indigo-develop/calibration_tools to calibrate the camera and generate CameraInfo.yaml.

----------
Record plant images
----------
Use DJI Go app and record Waypoints, see: http://forum.dji.com/thread-30373-1-1.html. You only need to do this for the first time. Every next time an already recorded track will be available in the history. We experienced that it is best to record 1 waypoint at the beginning and 1 waypoint at the end of the row. We flew with 0.2m/s.


----------
Convert plant images video into ROS bag
----------

----------
Localize the drone using visual markers
----------


----------
Segment plants and localize them
----------

----------
Plant matching
----------
