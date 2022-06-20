#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &laserCloudIn)
{
    pcl::fromROSMsg(*laserCloudIn,*laserCloud);
    cout<<laserCloud->points.size()<<endl;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"testMap");
    ros::NodeHandle nh;

    ros::Subscriber subMap = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround",5,mapCallback);

    ros::spin();
    return 0;
}