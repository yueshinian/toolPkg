/*
20210827
接收房间尺寸和里程计
4*13*1 = 52B/s
*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <mutex>
#include <memory>
#include <thread>
#include <iostream>
#include <chrono>
#include <unistd.h>

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <errno.h>
#include <stdlib.h>
#include <arpa/inet.h>

using namespace std;
using namespace cv;

mutex mt;
const int numPara = 13;
ros::Publisher pubRoomVis;
float recvFloatBuff[numPara];
bool isInitData = false;

class udpComm
{
private:
    struct sockaddr_in sendServerAddr;
    int sendSock;
    socklen_t sendAddrLen;
    char sendBuff[128];
    unsigned int sendServerPort;
    char *serverIp;
    bool usingUdp;

    struct sockaddr_in serverAddr;
    struct sockaddr_in clientAddr;
    int recvSock;
    socklen_t clientAddrLength;
    char recvBuff[128];
    unsigned int recvPort;
    size_t recvLength;

    bool closeUdp;
public:
    udpComm()
    {
        usingUdp = false;
        closeUdp = false;
    }

    ~udpComm() {}

    bool isUsingUdp()
    {
        return usingUdp;
    }

    void initSend(unsigned int sendServerPort_ = 7838, char *serverIp_ = "127.0.0.1")
    {
        serverIp = serverIp_;
        sendServerPort = sendServerPort_;
        /* 创建 socket , 关键在于这个 SOCK_DGRAM */
        if ((sendSock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
        {
            perror("socket");
            exit(errno);
        }
        else
        {
            std::cout << "create send sock" << std::endl;
        }

        memset(&sendServerAddr, 0, sizeof(sendServerAddr));
        /* 设置对方地址和端口信息 */
        sendServerAddr.sin_family = AF_INET;
        sendServerAddr.sin_port = htons(sendServerPort);
        sendServerAddr.sin_addr.s_addr = inet_addr(serverIp);
    }

    void initRecv(unsigned int recvPort_ = 7838)
    {
        recvPort = recvPort_;

        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(recvPort);
        serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        //serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

        if ((recvSock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
            perror("socket");
            exit(1);
        }
        else
        {
            printf("creat recv sock\n");
        }
        //port bind to server
        if (bind(recvSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
        {
            perror("bind");
            exit(1);
        }
        else
        {
            printf("bind success\n");
        }

        memset(&clientAddr, 0, sizeof(clientAddr));
        clientAddrLength = sizeof(clientAddr);
    }
    void recvMsg(char *recvBuff_, int len)
    {
        memset(recvBuff_, 0, sizeof(recvBuff_));
        recvLength = recvfrom(recvSock, recvBuff_, len, 0, (struct sockaddr *)&clientAddr, &clientAddrLength);
        if (recvLength < 0)
        {
            perror("recvfrom");
            exit(errno);
        }
        else
        {
            printf("success receive\n");
        }
    }
    int sendMsg(char *msg)
    {
        usingUdp = true;
        /* 发送UDP消息 */
        strcpy(sendBuff, msg);
        return sendto(sendSock, sendBuff, strlen(sendBuff), 0, (struct sockaddr *)&sendServerAddr, sizeof(sendServerAddr));
        usingUdp = false;
    }

    void floatArr2Bytes(float *floatArr, unsigned int len, char *byteBufOut)
    {
        unsigned int pos = 0;
        unsigned char *temp = nullptr;
        int k = 0;
        for (int i = 0; i < len; i++)
        {
            temp = (unsigned char *)(&floatArr[i]);

            for (k = 0; k < 4; k++)
            {
                byteBufOut[pos++] = *temp++;
            }
        }
    }

    void bytes2FloatArr(char *bytes, unsigned int len, float *floatArrOut)
    {
        unsigned int position = 0;
        unsigned int floatCount = 0;
        float *temp = nullptr;
        for (int i = 0; i < len;)
        {
            for (int k = 0; k < 4; k++)
            {
                temp = &floatArrOut[floatCount];
                *((unsigned char *)temp + k) = *(bytes + i);
                i++;
            }
            floatCount++;
        }
    }
};

void visualRoom(double minX_, double minY_, double maxX_, double maxY_, double minZ_, double maxZ_, string frame_id)
{
    visualization_msgs::Marker box;
    box.header.stamp = ros::Time::now();
    box.header.frame_id = frame_id;
    box.id = 1;
    box.ns = "boundary";
    box.type = visualization_msgs::Marker::LINE_LIST;
    box.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point box_p1, box_p2, box_p3, box_p4, box_p5, box_p6, box_p7, box_p8;
    box_p1.x = maxX_;
    box_p1.y = maxY_;
    box_p1.z = maxZ_;
    box_p2.x = minX_;
    box_p2.y = maxY_;
    box_p2.z = maxZ_;
    box_p3.x = minX_;
    box_p3.y = minY_;
    box_p3.z = maxZ_;
    box_p4.x = maxX_;
    box_p4.y = minY_;
    box_p4.z = maxZ_;
    box_p5.x = maxX_;
    box_p5.y = maxY_;
    box_p5.z = minZ_;
    box_p6.x = minX_;
    box_p6.y = maxY_;
    box_p6.z = minZ_;
    box_p7.x = minX_;
    box_p7.y = minY_;
    box_p7.z = minZ_;
    box_p8.x = maxX_;
    box_p8.y = minY_;
    box_p8.z = minZ_;

    box.scale.x = 0.1; //line width
    box.color.r = 0.0;
    box.color.g = 0.0;
    box.color.b = 255.0 / 255.0;

    box.color.a = 1;
    box.points.push_back(box_p1);
    box.points.push_back(box_p2);
    box.points.push_back(box_p2);
    box.points.push_back(box_p3);
    box.points.push_back(box_p3);
    box.points.push_back(box_p4);
    box.points.push_back(box_p4);
    box.points.push_back(box_p1);
    box.points.push_back(box_p1);
    box.points.push_back(box_p5);
    box.points.push_back(box_p2);
    box.points.push_back(box_p6);
    box.points.push_back(box_p3);
    box.points.push_back(box_p7);
    box.points.push_back(box_p4);
    box.points.push_back(box_p8);
    box.points.push_back(box_p5);
    box.points.push_back(box_p6);
    box.points.push_back(box_p6);
    box.points.push_back(box_p7);
    box.points.push_back(box_p7);
    box.points.push_back(box_p8);
    box.points.push_back(box_p8);
    box.points.push_back(box_p5);
    tf::Quaternion quat2;
    quat2.setEuler(0.0, 0.0, 0.0);
    box.pose.orientation.x = quat2.x();
    box.pose.orientation.y = quat2.y();
    box.pose.orientation.z = quat2.z();
    box.pose.orientation.w = quat2.w();
    box.lifetime = ros::Duration(0.0);
    box.frame_locked = false;
    pubRoomVis.publish(box);

    visualization_msgs::Marker textX;
    textX.header.frame_id = frame_id;
    textX.header.stamp = ros::Time::now();
    textX.ns = "textX";
    textX.action = visualization_msgs::Marker::ADD;
    textX.pose.orientation.w = 1.0;
    textX.id = 0;
    textX.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textX.scale.z = 1.0;
    textX.color.b = 0;
    textX.color.g = 0;
    textX.color.r = 1;
    textX.color.a = 1;
    textX.pose.position.x = (maxX_ + minX_) / 2;
    textX.pose.position.y = maxY_ - 1;
    textX.pose.position.z = maxZ_;
    textX.text = "SIZE_X: " + to_string(abs(maxX_) + abs(minX_)) + "=" + to_string(abs(minX_)) + "+" + to_string(abs(maxX_));
    pubRoomVis.publish(textX);

    visualization_msgs::Marker textY;
    textY.header.frame_id = frame_id;
    textY.header.stamp = ros::Time::now();
    textY.ns = "textY";
    textY.action = visualization_msgs::Marker::ADD;
    textY.pose.orientation.w = 1.0;
    textY.id = 0;
    textY.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textY.scale.z = 1;
    textY.color.b = 0;
    textY.color.g = 0;
    textY.color.r = 1;
    textY.color.a = 1;
    textY.pose.position.x = maxX_ - 2;
    textY.pose.position.y = (maxY_ + minY_) / 2;
    textY.pose.position.z = maxZ_;
    textY.text = "SIZE_Y: " + to_string(abs(maxY_) + abs(minY_)) + "=" + to_string(abs(minY_)) + "+" + to_string(abs(maxY_));
    pubRoomVis.publish(textY);
}

tf::StampedTransform odomTrans;
tf::TransformBroadcaster tfBroadcasterPointer;

void pubTf()
{
    ros::Rate r(20);
    while (ros::ok())
    {
        if (!isInitData)
        {
            continue;
        }
        // publish tf messages
        odomTrans.stamp_ = ros::Time::now();
        odomTrans.frame_id_ = "/map";
        odomTrans.child_frame_id_ = "/velodyne";
        odomTrans.setRotation(tf::Quaternion(recvFloatBuff[9], recvFloatBuff[10], recvFloatBuff[11], recvFloatBuff[12]));
        odomTrans.setOrigin(tf::Vector3(recvFloatBuff[6], recvFloatBuff[7], recvFloatBuff[8]));
        tfBroadcasterPointer.sendTransform(odomTrans);
    }
}

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "roomUdpServer");
        ros::NodeHandle nh;

        pubRoomVis = nh.advertise<visualization_msgs::Marker>("roomVis", 1);
        ros::Publisher pubOdom = nh.advertise<nav_msgs::Odometry>("robotOdom", 5);
        auto subRoomUdp = make_shared<udpComm>();
        subRoomUdp->initRecv(12020);

        char recvBuf[4 * numPara];
        float recvRoom[numPara];
        nav_msgs::Odometry odom;
        while (ros::ok())
        {
            subRoomUdp->recvMsg(recvBuf, 4 * numPara);
            subRoomUdp->bytes2FloatArr(recvBuf, 4 * numPara, recvFloatBuff);
            odom.pose.pose.position.x = recvFloatBuff[6];
            odom.pose.pose.position.y = recvFloatBuff[7];
            odom.pose.pose.position.z = recvFloatBuff[8];
            odom.pose.pose.orientation.x = recvFloatBuff[9];
            odom.pose.pose.orientation.y = recvFloatBuff[10];
            odom.pose.pose.orientation.z = recvFloatBuff[11];
            odom.pose.pose.orientation.w = recvFloatBuff[12];
            odom.header.frame_id = "/map";
            pubOdom.publish(odom);

            tf::StampedTransform transformToMap;
            transformToMap.setOrigin(tf::Vector3(recvFloatBuff[6], recvFloatBuff[7], recvFloatBuff[8]));
            transformToMap.setRotation(tf::Quaternion(recvFloatBuff[9], recvFloatBuff[10], recvFloatBuff[11], recvFloatBuff[12]));
            tf::Vector3 vec;

            for (int i = 0; i < 2; ++i)
            {
                int signal = (i == 0 ? -1 : 1);
                vec.setX(signal * recvFloatBuff[i * 3 + 0]);
                vec.setY(signal * recvFloatBuff[i * 3 + 1]);
                vec.setZ(signal * recvFloatBuff[i * 3 + 2]);
                vec = transformToMap * vec;
                recvRoom[i * 3 + 0] = vec.x();
                recvRoom[i * 3 + 1] = vec.y();
                recvRoom[i * 3 + 2] = vec.z();
            }
            std::cout <<"roomXYZXYZ velodyne "
                      << -recvFloatBuff[0] << ' ' << -recvFloatBuff[1] << ' ' << -recvFloatBuff[2] << ' '
                      << recvFloatBuff[3] << ' ' << recvFloatBuff[4] << ' ' << recvFloatBuff[5] << std::endl
                      <<"roomXYZXYZ map "
                      << recvRoom[0] << ' ' << recvRoom[1] << ' ' << recvRoom[2] << ' '
                      << recvRoom[3] << ' ' << recvRoom[4] << ' ' << recvRoom[5] << std::endl;
            visualRoom(-recvFloatBuff[0], -recvFloatBuff[1], -recvFloatBuff[2], recvFloatBuff[3], recvFloatBuff[4], recvFloatBuff[5], "/velodyne");
            visualRoom(recvRoom[0], recvRoom[1], recvRoom[2], recvRoom[3], recvRoom[4], recvRoom[5], "/map");
            isInitData = true;
        }
        ros::waitForShutdown();
        //ros::spin();
    }