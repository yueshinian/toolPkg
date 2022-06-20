/*
20210827
发送房间尺寸和里程计
4*13*1 = 52B/s
*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

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

public:
    udpComm()
    {
        usingUdp = false;
    }

    ~udpComm() {}

    bool isUsingUdp(){
        return usingUdp;
    }

    void initSend(unsigned int sendServerPort_ = 7838, char* serverIp_= "127.0.0.1")
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

    void initRecv(unsigned int recvPort_=7838)
    {
        recvPort = recvPort_;

        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(recvPort);
        serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

        if ((recvSock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
            perror("socket");
            exit(1);
        }
        else
        {
            printf("creat recv sock");
        }
        //port bind to server
        if (bind(recvSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
        {
            perror("bind");
            exit(1);
        }

        memset(&clientAddr, 0, sizeof(clientAddr));
        clientAddrLength = sizeof(clientAddr);
    }
    void recvMsg()
    {
        while (1)
        {
            memset(recvBuff, 0, sizeof(recvBuff));
            recvLength = recvfrom(recvSock, recvBuff, sizeof(recvBuff) - 1, 0, (struct sockaddr *)&clientAddr, &clientAddrLength);
            if (recvLength < 0)
            {
                perror("recvfrom");
                exit(errno);
            }
            else
            {
                std::cout << recvBuff << std::endl;
            }
        }
    }
    int sendMsg(char *msg, int len)
    {
        usingUdp = true;
        /* 发送UDP消息 */
        //strcpy(sendBuff, msg);
        int sendState = sendto(sendSock, (char *)msg, len, 0, (struct sockaddr *)&sendServerAddr, sizeof(sendServerAddr));
        usingUdp = false;
        return sendState;
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

shared_ptr<udpComm> pubRoomUdp;
nav_msgs::Odometry odom;
int pclCnt=0;
const int numPara = 13;
void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &laserCloudIn)
{
    if(pclCnt==0){
        pclCnt=10;
    }else{
        --pclCnt;
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*laserCloudIn,*laserCloud);

    float minX = 0;
    float minY = 0;
    float maxX = 0;
    float maxY = 0;

    pcl::PointXYZI thisPoint;

    int cloudSize = laserCloud->points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
        thisPoint.x = laserCloud->points[i].x;
        thisPoint.y = laserCloud->points[i].y;
        thisPoint.z = laserCloud->points[i].z;

        //measure room
        float angle = atan2(thisPoint.y, thisPoint.x) / M_PI * 180;
        float rangeAngle = 1;
        if (angle > 0 - rangeAngle && angle < 0 + rangeAngle)
        {
            maxX = max(maxX, abs(thisPoint.x));
        }
        else if (angle > 90 - rangeAngle && angle < 90 + rangeAngle)
        {
            maxY = max(maxY, abs(thisPoint.y));
        }
        else if (angle > -90 - rangeAngle && angle < -90 + rangeAngle)
        {
            minY = max(minY, abs(thisPoint.y));
        }
        else if (angle > 180 - rangeAngle || angle < -180 + rangeAngle)
        {
            minX = max(minX, abs(thisPoint.x));
        }
    }
    if (minX == 0 && minY == 0 && maxX == 0 && maxY == 0)
        return;
    char charArry[4*numPara];
    float floatArry[numPara]  = {minX, minY, 0,maxX, maxY,1,odom.pose.pose.position.x,
                                                            odom.pose.pose.position.y,odom.pose.pose.position.z,
                                                            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w};
    pubRoomUdp->floatArr2Bytes(floatArry,numPara,charArry);
    while(pubRoomUdp->isUsingUdp());
    if(pubRoomUdp->sendMsg(charArry,4*numPara)){
        cout<<"send msg success!"<<endl;
        cout<<minX<<' '<< minY<<' '<< maxX<<' '<< maxY<<endl;
    }else{
        cout<<"send msg failed!"<<endl;
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odomData)
{
    lock_guard<mutex> lg(mt);
    odom = *odomData;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roomUdpClient");
    ros::NodeHandle nh;

    pubRoomUdp = make_shared<udpComm>();
    pubRoomUdp->initSend(12020,"127.0.0.1");

    ros::Subscriber subVelodyne = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, velodyneCallback);
    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 10, odomCallback);

    ros::spin();
}