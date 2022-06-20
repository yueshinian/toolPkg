// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <memory>
#include <cmath>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>

const double PI = 3.1415926;

//lym
ros::Publisher pubCloudMap;
ros::Publisher pubLaserCloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCLoudInMapFrame(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCLoudInMapFrame2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudMap(new pcl::PointCloud<pcl::PointXYZI>());
void cloudMapHandle( const sensor_msgs::PointCloud2ConstPtr& laserCloud){
  laserCloudMap->clear();
  pcl::fromROSMsg(*laserCloud, *laserCloudMap);
  pubCloudMap.publish(*laserCloudMap);
}
int laserodomcount=0;
void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,  const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  if(laserodomcount>0){
    --laserodomcount;
    return;
  }else{
    ++laserodomcount;
  }
  laserCloudIn->clear();
  laserCLoudInMapFrame->clear();
  laserCLoudInMapFrame2->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  pcl::PointXYZI p1,p2;
  tf::Vector3 vec;
 int laserCloudInNum = laserCloudIn->points.size();
  pcl::PointXYZI point;
  for (int i = 0; i < laserCloudInNum; i++){
    point = laserCloudIn->points[i];
    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;
    float angle = atan2(pointY,pointX )/PI*180;
    float dis = sqrt((pointX * pointX ) + (pointY  * pointY ));//lym
    if ( (dis<0.75
       && ( (angle>30 && angle<55) 
       || (angle>123 && angle<143) 
       || (angle<=-124 &&  angle>=-151) 
       || (angle<-31 && angle>-58)  ) 
       && pointZ<0.1) ||
       ((angle<20 && angle >-20) && dis<0.60 && pointZ<0.1))   continue;
    p1.x = pointY;
    p1.y = pointZ;
    p1.z = pointX;
    p1.intensity=laserCloudIn->points[i].intensity;
    laserCLoudInMapFrame2->points.push_back(p1);
  }
 
    tf::StampedTransform transformToMap;
    transformToMap.setOrigin(tf::Vector3(odometry->pose.pose.position.x, odometry->pose.pose.position.y, odometry->pose.pose.position.z));
    transformToMap.setRotation(tf::Quaternion(odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w));
    
   
    for (int i = 0; i < laserCloudInNum; i++){
    p1 = laserCLoudInMapFrame2->points[i];
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);

    vec = transformToMap * vec;
    
    p2.x = vec.z();
    p2.y = vec.x();
    p2.z = vec.y();
    
    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();
    p1.intensity=laserCloudIn->points[i].intensity;
    laserCLoudInMapFrame->points.push_back(p1);
  }
  sensor_msgs::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInMapFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "/map";
  pubLaserCloud.publish(scan_data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loamInterfacenode");
    ros::NodeHandle nh;
    pubCloudMap = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/point_cloud_raw",1);
    //ros::Subscriber subCloudMap = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_map", 1, cloudMapHandle); 
    //lym
    // ROS message filters
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_registered_cloud2", 5);
    message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
    typedef message_filters::Synchronizer<syncPolicy> Sync; 
    boost::shared_ptr<Sync> sync_;
    subOdometry.subscribe(nh, "/integrated_to_init", 10);
    subLaserCloud.subscribe(nh, "/velodyne_points", 10);
    sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
    sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2)); 

    ros::spin();

    return 0;
}

