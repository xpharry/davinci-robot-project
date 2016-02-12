//cwru_pcl_utils_example_main.cpp
// show use of CwruPclUtils library, using public functions:
//   save_kinect_snapshot();
//   transformTFToEigen();
//   transform_kinect_cloud();
//   save_transformed_kinect_snapshot();
// run this w/:
// roslaunch cwru_baxter_sim baxter_world.launch
// roslaunch cwru_baxter_sim kinect_xform.launch
// rosrun cwru_pcl_utils 

#include <cwru_pcl_utils/cwru_pcl_utils.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "cwru_pcl_utils_davinci_main"); //node name
    ros::NodeHandle nh;
    CwruPclUtils cwru_pcl_utils(&nh);

    // whole point cloud received including coordinate information
    while (!cwru_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");
    ROS_INFO("saving pointcloud");
    cwru_pcl_utils.save_kinect_snapshot();
    cwru_pcl_utils.save_kinect_clr_snapshot();  // save color version of pointcloud as well


    //set up a publisher to display clouds in rviz:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_cloud_display", 1);
    //pcl::PointCloud<pcl::PointXYZ> & outputCloud
    pcl::PointCloud<pcl::PointXYZ> display_cloud; // instantiate a pointcloud object, which will be used for display in rviz
    sensor_msgs::PointCloud2 pcl2_display_cloud; //(new sensor_msgs::PointCloud2); //corresponding data type for ROS message


    Eigen::Vector3f centroid;
    // select point(s) and compute the centroid
    while (ros::ok()) {
        if (cwru_pcl_utils.got_selected_points()) {
            ROS_INFO("process selected-points!");

            centroid = cwru_pcl_utils.get_centroid(cwru_pcl_utils.get_selected_points());
            cwru_pcl_utils.reset_got_selected_points();   // reset the selected-points trigger
        }

        pcl::toROSMsg(display_cloud, pcl2_display_cloud); //convert datatype to compatible ROS message type for publication
        pcl2_display_cloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
        pubCloud.publish(pcl2_display_cloud); //publish a point cloud that can be viewed in rviz (under topic pcl_cloud_display)

        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }
    ROS_INFO("my work here is done!");

}

