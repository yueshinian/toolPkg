#include <ros/ros.h>
#include "octomap_world/octomap_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generateOctomap");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    volumetric_mapping::OctomapManager *manager_ = new volumetric_mapping::OctomapManager(nh, nh_private);
    ros::spin();
    return 0;
}