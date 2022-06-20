#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <mutex>
#include <vector>
#include <queue>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <atomic>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <omp.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

class pointFrameTrans
{
    using PointType = pcl::PointXYZI;

public:
    pointFrameTrans()
    {
        laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCLoudInMapFrame.reset(new pcl::PointCloud<pcl::PointXYZI>());
        odomTime = 0;
        pointTime = 0;
        //tf::TransformBroadcaster tfBroadcaster;
        //tfBroadcasterPointer = &tfBroadcaster;
        subVelodynePoints = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &pointFrameTrans::laserCloudHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init", 10, &pointFrameTrans::odomHandler, this, ros::TransportHints().tcpNoDelay());
        pubPointMapFrame = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 10);
        pubOdometry = nh.advertise<nav_msgs::Odometry>("/state_estimation", 10);
        pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/sensor_scan", 10);
    }

    ~pointFrameTrans() {}

    void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &laserCloudIn)
    {
        // ROS_INFO("velodynePoints");
        std::lock_guard<std::mutex> lg(pointMt);
        pointQueue.emplace(*laserCloudIn);
        // while(!pointQueue.empty()){
        //     double time = pointQueue.front().header.stamp.toSec();
        //     if(time - odomTime > 50.0){
        //         pointQueue.pop();
        //     }
        // }
        if (pointQueue.size() > 100)
        {
            pointQueue.pop();
        }
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, nav_msgs::Odometry odom)
    {
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transCur = pcl::getTransformation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, roll, pitch, yaw);

        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    void odomHandler(const nav_msgs::Odometry::ConstPtr &odom)
    {
        // initSystem = true;
        odomTime = odom->header.stamp.toSec();
        while (!pointQueue.empty())
        {
            pointTime = pointQueue.front().header.stamp.toSec();
            if (pointTime - odomTime <= -0.01)
            {
                std::cout << "delta time is: " << pointTime - odomTime << std::endl;
                pointQueue.pop();
            }
            else
            {
                break;
            }
        }
        if (!pointQueue.empty())
        {
            ROS_INFO("TRUE1");
            //转换坐标系
            double roll, pitch, yaw;
            geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
            odomData = *odom;

            if (flipStateEstimation)
            {
                tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

                pitch = -pitch;
                yaw = -yaw;

                geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

                odomData.pose.pose.orientation = geoQuat;
                odomData.pose.pose.position.x = odom->pose.pose.position.z;
                odomData.pose.pose.position.y = odom->pose.pose.position.x;
                odomData.pose.pose.position.z = odom->pose.pose.position.y;
            }

            // publish odometry messages
            odomData.header.frame_id = "/map";
            odomData.child_frame_id = "/sensor";
            pubOdometry.publish(odomData);

            // publish tf messages
            odomTrans.stamp_ = odom->header.stamp;
            odomTrans.frame_id_ = "/map";
            odomTrans.child_frame_id_ = "/sensor";
            odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
            odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

            if (sendTF)
            {
                if (!reverseTF)
                {
                    tfBroadcasterPointer.sendTransform(odomTrans);
                }
                else
                {
                    tfBroadcasterPointer.sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "/sensor", "/map"));
                }
            }
            //转换点云
            ROS_INFO("TRUE2");
            sensor_msgs::PointCloud2 velodynePoints2;
            velodynePoints2 = pointQueue.front();
            velodynePoints2.header.frame_id = "sensor";
            velodynePoints2.header.stamp = odom->header.stamp;
            pubLaserCloud.publish(velodynePoints2);
            laserCloudIn->clear();
            pcl::fromROSMsg(pointQueue.front(), *laserCloudIn);
            pointQueue.pop();
            // laserCloudIn = transformPointCloud(laserCloudIn, *odom);
            tf::StampedTransform transformToMap;
            transformToMap.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));
            transformToMap.setRotation(tf::Quaternion(odomData.pose.pose.orientation.x, odomData.pose.pose.orientation.y, odomData.pose.pose.orientation.z, odomData.pose.pose.orientation.w));
            pcl::PointXYZI p1;
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
                p1.intensity = laserCloudIn->points[i].intensity;
                laserCLoudInMapFrame->points.push_back(p1);
            }
            sensor_msgs::PointCloud2 velodynePoints;
            pcl::toROSMsg(*laserCLoudInMapFrame, velodynePoints);
            velodynePoints.header.stamp = odom->header.stamp;
            velodynePoints.header.frame_id = "/map";
            pubPointMapFrame.publish(velodynePoints);
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pubPointMapFrame;
    ros::Publisher pubOdometry;
    ros::Publisher pubLaserCloud;
    ros::Subscriber subVelodynePoints;
    ros::Subscriber subOdom;
    double pointTime;
    double odomTime;
    std::queue<nav_msgs::Odometry> odomQueue;
    std::queue<sensor_msgs::PointCloud2> pointQueue;
    std::mutex pointMt;
    std::mutex odomMt;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCLoudInMapFrame;
    // std::atomic<bool> initSystem;
    nav_msgs::Odometry odomData;
    tf::StampedTransform odomTrans;
    tf::TransformBroadcaster tfBroadcasterPointer;
    const bool flipStateEstimation = true;
    const bool flipRegisteredScan = true;
    const bool sendTF = true;
    const bool reverseTF = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointFrameTrans");
    pointFrameTrans *mainPtr = new pointFrameTrans();
    ros::spin();
    return 0;
}