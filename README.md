# DaVinci_Robot
stuff related to DaVinci Surgical Robot

***********************************************************************

roslaunch dvrk_model derlin_davinci_gazebo.launch

rosrun rqt_reconfigure rqt_reconfigure

rosrun davinci_kinematics test_pointer        (this is the pointer test node)

	Then, you can test points manually, e.g. with:

		rostopic pub  /thePoint geometry_msgs/Point  '{x: -0.1, y: 0.0, z: 0.1}'
   
	Can check the result with:

		rosrun tf tf_echo left_camera_optical_frame one_tool_tip_link

rosrun davinci_pcl_utils davinci_pcl_utils_main2

	use "Publish Selected Points" to grasp a patch of points and then the gripper will to the centroid

	viable exitpoints would be diplayed in pink markers

	use "Publish Point" to click any pink marker and it will become blue which represents your final exit point	


***************************************************************************

*********
In the needle_planner package, see the code: test_needle_plan_horiz_kvec.cpp.  This code prompts for an entrance point (x,y,z) and a yaw angle (which it uses to compute an exit point).  It packages the entrance and exit points in a geometry_msgs/Polygon message and publishes to "/entrance_and_exit_pts".

Another node, "needle_plan_horiz_kvec" (which is launched by the updated launch file, derlin_davinci_gazebo.launch) subscribes to this topic and automatically generates a *.csp file.
You can then run this trajectory file with:
rosrun playfile_reader playfile_cameraspace gripper_poses_in_camera_coords.csp
*********

3-22

marker model change

	remove colision feature
