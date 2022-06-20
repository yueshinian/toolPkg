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
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Int32.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

//lym
const double PI = 3.1415926;
const double exploredAreaVoxelSize = 0.3;
const bool useCloudRing = false;
const float sensorMinimumRange = 0.4;
// VLP-16
const int N_SCAN = 16;
const int Horizon_SCAN = 1800;
const float ang_res_x = 0.2;
const float ang_res_y = 2.0;
const float ang_bottom = 15.0 + 0.1;

ros::Publisher pubVelodyneInMap;
ros::Publisher pubVelodyneHistory;

nav_msgs::Odometry odometry;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudInMapFrame(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudHistory(new pcl::PointCloud<pcl::PointXYZ>());

mutex mt;

pcl::VoxelGrid<pcl::PointXYZ> sor;
bool newOdom;

void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &laserCloud2)
{
  laserCloudIn->clear();
  laserCloudInMapFrame->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);
  tf::StampedTransform transformToMap;
  // transformToMap.setOrigin(tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z));
  // transformToMap.setRotation(tf::Quaternion(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w));

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odometry.pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
  transformToMap.setOrigin(tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y, 0));
  transformToMap.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));

  pcl::PointXYZ p1;
  tf::Vector3 vec;
  int laserCloudInNum = laserCloudIn->points.size();
  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);
    vec = transformToMap * vec;
    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();
    laserCloudInMapFrame->points.push_back(p1);
  }
  sensor_msgs::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCloudInMapFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "/map";
  pubVelodyneInMap.publish(scan_data);

  if (newOdom)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudHistoryTemp(new pcl::PointCloud<pcl::PointXYZ>());
    *laserCloudHistory += *laserCloudInMapFrame;
    sor.setInputCloud(laserCloudHistory); //输入
    sor.setLeafSize(0.3f, 0.3f, 0.3f);    //分别率,越小越密,参数分别是xyz
    sor.filter(*laserCloudHistory);   //输出
    //laserCloudHistory = laserCloudHistoryTemp;
    sensor_msgs::PointCloud2 mapPoints;
    pcl::toROSMsg(*laserCloudHistory, mapPoints);
    mapPoints.header.stamp = laserCloud2->header.stamp;
    mapPoints.header.frame_id = "/map";
    pubVelodyneHistory.publish(mapPoints);
    newOdom = false;
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  //odometry = *msg;
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  odometry = *odom;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
  pitch = -pitch;
  yaw = -yaw;
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  odometry.pose.pose.orientation = geoQuat;
  odometry.pose.pose.position.x = odom->pose.pose.position.z;
  odometry.pose.pose.position.y = odom->pose.pose.position.x;
  odometry.pose.pose.position.z = odom->pose.pose.position.y;
  newOdom = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "createVelodyneInMap");
  ros::NodeHandle nh;

  odometry.header.frame_id = "/map";
  odometry.pose.pose.position.x = 0;
  odometry.pose.pose.position.y = 0;
  odometry.pose.pose.position.z = 0;
  odometry.pose.pose.orientation.x = 0;
  odometry.pose.pose.orientation.y = 0;
  odometry.pose.pose.orientation.z = 0;
  odometry.pose.pose.orientation.w = 1;

  newOdom = false;
  pubVelodyneInMap = nh.advertise<sensor_msgs::PointCloud2>("/velodyneInMap", 2);
  pubVelodyneHistory = nh.advertise<sensor_msgs::PointCloud2>("/velodyneHistoryInMap", 2);
  ros::Subscriber subVelodyne = nh.subscribe("/velodyne_points", 10, velodyneCallback);
  ros::Subscriber subOdom = nh.subscribe("/integrated_to_init", 10, odomCallback);

  ros::spin();
  return 0;
}
