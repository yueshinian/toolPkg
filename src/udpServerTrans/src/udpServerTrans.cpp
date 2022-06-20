#include <ros/ros.h>
#include <cupdpsocket.h>
#include <pthread.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace cv;
using namespace std;

const double PI = 3.1415926;

ros::Publisher pubVelodynePoint;
/****************************传输修改图像参数**************************/
#define LOOP_TIME 0.05
#define RGB_WIDTH 640			   // 接收RGB图像的宽
#define RGB_HEIGHT 480			   // 接收RGB图像的高
#define DEPTH_WIDTH 1800			   // 接收深度图像的宽
#define DEPTH_HEIGHT 16		   // 接收深度图像的高
#define ONE_DEPTH_VALUE_BYTE_NUM 4 //一个深度值占的字节数   32位占4个 16位占2个  //不同的深度值修改这里

#define MIN_TRANS_BYTE_NUM 50000					   //最小传输字节数量
Mat recvRGBimg(RGB_HEIGHT, RGB_WIDTH, CV_8UC3);		   //接收彩色图
Mat recvDepthimg(DEPTH_HEIGHT, DEPTH_WIDTH, CV_32FC1); //接收深度图
/******************************************************************/

struct Sock_info
{
	int m_fd;
	pthread_t m_pthid;
	struct sockaddr_in m_addr;
};
//char数组转float数组
void bytes2FloatArr(unsigned char *bytes, unsigned int len, float *floatArrOut)
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
//char数组转ushort数组
void bytes2UShortArr(unsigned char *bytes, unsigned int len, ushort *floatArrOut)
{
	unsigned int position = 0;
	unsigned int floatCount = 0;
	ushort *temp = nullptr;
	for (int i = 0; i < len;)
	{
		for (int k = 0; k < 2; k++)
		{
			temp = &floatArrOut[floatCount];
			*((unsigned char *)temp + k) = *(bytes + i);
			i++;
		}
		floatCount++;
	}
}
image_transport::Publisher pubRGBPicture;	//发布RGB图像
image_transport::Publisher pubDepthPicture; //发布深度图像
//udp通讯类
CUdpSocket g_pudp;
//接收缓冲区
unsigned char recvBuf[65536];

/*******************深度值接收变量******************/
//深度值缓冲
vector<float> depthValueBuf; //不同的深度值修改这里
//深度值分包接收缓冲
float depthValueSubBuf[MIN_TRANS_BYTE_NUM]; //不同的深度值修改这里
//接收深度值计数
int recvDepthCnt = 0;
/*********************rgb接收变量******************/
//rgb缓冲
vector<uchar> rgbValueBuf;
//接收深度值计数
int recvRGBCnt = 0;
/*************************************************/
//接收线程退出标志位
bool threadFinishFlag = false;
void *receiveSocketThread(void *ptr)
{
	while (!threadFinishFlag)
	{
		g_pudp.OnReceive();
		//接收到数据
		if (g_pudp.isRecvData)
		{
			ROS_INFO("%d", g_pudp.recvLen);
			for (int i = 0; i < g_pudp.recvLen; i++)
			{
				recvBuf[i] = g_pudp.recvCharArray[i];
			}
			if ('d' == recvBuf[0] && 'e' == recvBuf[1] && 'p' == recvBuf[2])
			{
				if (recvBuf[3] != '!')
				{
					// ROS_INFO("%d",recvBuf[3]);
					if (recvBuf[3] == recvDepthCnt)
					{
						bytes2FloatArr(recvBuf + 4, g_pudp.recvLen - 4, depthValueSubBuf);
						// bytes2UShortArr(recvBuf+4,g_pudp.recvLen-4,depthValueSubBuf); //不同的深度值修改这里
						for (int i = 0; i < (g_pudp.recvLen - 4) / (ONE_DEPTH_VALUE_BYTE_NUM * 1.0); i++)
						{
							depthValueBuf.push_back(depthValueSubBuf[i]);
						}
						recvDepthCnt++;
					}
					else
					{
						ROS_ERROR("depth picture  assembly error!");
					}
				}
				else //最后一帧
				{
					recvDepthCnt = 0;
					bytes2FloatArr(recvBuf + 4, g_pudp.recvLen - 4, depthValueSubBuf);
					// bytes2UShortArr(recvBuf+4,g_pudp.recvLen-4,depthValueSubBuf);//不同的深度值修改这里
					for (int i = 0; i < (g_pudp.recvLen - 4) / (ONE_DEPTH_VALUE_BYTE_NUM * 1.0); i++)
					{
						depthValueBuf.push_back(depthValueSubBuf[i]);
					}
					ROS_INFO("%d", depthValueBuf.size());
					for (int i = 0; i < DEPTH_HEIGHT; i++)
					{
						for (int j = 0; j < DEPTH_WIDTH; j++)
						{
							recvDepthimg.at<float>(i, j) = depthValueBuf[i * DEPTH_WIDTH + j]; //不同的深度值修改这里
						}
					}

					//imshow("server0", recvDepthimg);
					 //waitKey(1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr velodyneCloud(new pcl::PointCloud<pcl::PointXYZ>());
					pcl::PointXYZ point;

					for (int i = 0; i < 16; ++i)
					{
						for (int j = 0; j < 1800; ++j)
						{
							float range = recvDepthimg.at<float>(i, j);
							if (range <= 0)
								continue;
							double angleZ = (i * 2.0 - 15.0) * PI / 180.0;
							double angleXY = (j * 0.2 - 180.0 + 0.2) * PI / 180.0;
							point.z = range * sin(angleZ);
							point.x = range * cos(angleZ) * cos(angleXY);
							point.y = range * cos(angleZ) * sin(angleXY);
							velodyneCloud->points.emplace_back(point);
						}
					}

					sensor_msgs::PointCloud2 velodynePoints;
					pcl::toROSMsg(*velodyneCloud, velodynePoints);
					velodynePoints.header.frame_id = "/velodyne";
					pubVelodynePoint.publish(velodynePoints);
				}
			}
			if ('r' == recvBuf[0] && 'g' == recvBuf[1] && 'b' == recvBuf[2])
			{
				if (recvBuf[3] != '!')
				{
					if (recvBuf[3] == recvRGBCnt)
					{
						for (int i = 4; i < g_pudp.recvLen; i++)
						{
							rgbValueBuf.push_back(recvBuf[i]);
						}
						recvRGBCnt++;
					}
					else
					{
						ROS_ERROR("depth picture  assembly error！");
					}
				}
				else //最后一帧
				{
					recvRGBCnt = 0;
					for (int i = 4; i < g_pudp.recvLen; i++)
					{
						rgbValueBuf.push_back(recvBuf[i]);
					}
					ROS_INFO("%d", rgbValueBuf.size());
					recvRGBimg = imdecode(rgbValueBuf, IMREAD_COLOR); //图像解码
					sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", recvRGBimg).toImageMsg();
					pubRGBPicture.publish(msg);
					imshow("server1", recvRGBimg);
					waitKey(1);
					rgbValueBuf.clear();
				}
			}
		}
	}
	pthread_exit(NULL);
}
int main(int argc, char **argv)
{
	/************************Initiate ROS***********************/
	ros::init(argc, argv, "udpServerTrans"); //初始化节点
	ros::AsyncSpinner spinner(1);			 //开启线程
	spinner.start();
	ros::NodeHandle nh;

	//发布RGB图像和彩色图像
	image_transport::ImageTransport it(nh);
	/***********************************************************/
	pubRGBPicture = it.advertise("camer/rgb", 1);
	pubDepthPicture = it.advertise("camer/depth", 1);

	pubVelodynePoint = nh.advertise<sensor_msgs::PointCloud2>("/velodynePoints", 1);
	//创建udp接收线程
	pthread_t receiveDataId;
	if (pthread_create(&receiveDataId, NULL, receiveSocketThread, NULL))
	{
		cout << "create createThread error!" << endl;
		return 0;
	}

	while (ros::ok())
	{
		//起始时间
		double start = ros::Time::now().toSec();
		// //接收指令数据
		// ros::spinOnce();
		double sleep_time = LOOP_TIME - (ros::Time::now().toSec() - start);
		if (sleep_time > 0)
		{
			ros::Duration(sleep_time).sleep();
			//ROS_INFO("Cost Time: %lf ms", (LOOP_TIME-sleep_time)*1000);
		}
		else
			ROS_WARN("control loop over time: %f ms", -sleep_time * 1000);
	}
	threadFinishFlag = true;
	ros::waitForShutdown();
	return 0;
}