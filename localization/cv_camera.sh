# rosparam set cv_camera/device_id 0
rosparam set cv_camera/image_width 1280
rosparam set cv_camera/image_height 720
# rosrun topic_tools relay cv_camera/image_raw camera/image_raw &
# rosrun cv_camera cv_camera_node
rosrun cv_camera cv_camera_node __name:=camera1 _device_id:=0
# rosrun cv_camera cv_camera_node _device_id:=0
