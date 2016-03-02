
$    roslaunch cwru_davinci_vision davinci_endo.launch 

$    roslaunch cwru_davinci_vision davinci_endo_view.launch

$    rosrun rviz rviz

no point cloud? check usb_stereo.launch, the camera_frame_id for left and right should be the same

<param name="camera_frame_id" value="davinci_endo" />

rosrun rqt_reconfigure rqt_reconfigure to play with parameters. 


checkerboard 


rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 9x6 --square 0.013 right:=/davinci_endo/right/image_color left:=/davinci_endo/left/image_color right_camera:=/davinci_endo/unsynced/right left_camera:=/davinci_endo/unsynced/left


roslaunch cwru_davinci_vision usb_calibration.launch 
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 9x6 --square 0.013 right:=/davinci_endo/right/image_raw left:=/davinci_endo/left/image_raw right_camera:=/davinci_endo/right left_camera:=/davinci_endo/left


rosrun camera_calibration cameracalibrator.py --approximate 0.01 --size 7x6 --square 0.20 right:=/davinci/right_camera/image_raw left:=/davinci/left_camera/image_raw right_camera:=/davinci/right_camera left_camera:=/davinci/left_camera
