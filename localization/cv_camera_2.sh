# rosparam set cv_camera/device_id 1
rosparam set cv_camera/image_width 1280
rosparam set cv_camera/image_height 720
# rosrun topic_tools relay cv_camera/image_raw camera_two/image_raw &
# rosrun cv_camera cv_camera_node __name:=camera2
rosrun cv_camera cv_camera_node __name:=camera2 _device_id:=1

