#include "realsense_basics/RealsenseCamera.h"

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "realsense_camera_node");

    RealsenseCamera camera;

    while(ros::ok())
    {
        camera.publishPointcloud();
        ros::spinOnce();
    }  

    return 0;
}