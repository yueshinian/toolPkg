#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
// #include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace std;

#define LOOP_TIME   0.05
#define MIN_TRANS_BYTE_NUM 50000  //最小传输字节数量 
struct sockaddr_in serveradr;
int sock;
//彩色图分包缓存
uchar rgbSubPackageBuf[MIN_TRANS_BYTE_NUM+4]; 
//深度图分包缓存
uchar depthSubPackageBuf[MIN_TRANS_BYTE_NUM+4];
//判断udp是否被占用 
bool isUdpUsage = false; 
void SendData(unsigned char *buf ,int len)
{
    isUdpUsage = true;
    int ret = sendto(sock,(char*)buf,len,0,(struct sockaddr*)&serveradr,sizeof(sockaddr_in));
    if(ret <0)
    {
       printf("send error!");
    }
    isUdpUsage = false;
}
void rgbImage_socket(Mat inImg,char head[3])
{
    //imshow("image_socket", inImg);//显示图片
    //waitKey(1);
    if( inImg.empty() )
    {
      ROS_INFO("Camera image empty");
      return;//break;
    }
    //图像压缩
    Mat image = inImg;
    vector<uchar> image_encode;  
    vector<int> quality;  
    quality.push_back(IMWRITE_JPEG_QUALITY);  
    quality.push_back(50);//进行50%的压缩  
    imencode(".jpg", image, image_encode,quality);//将图像编码  
    int rgbSize = image_encode.size(); 
    ROS_INFO("%d",rgbSize); 
    for(int i = 0;i<3;i++)
    {
        rgbSubPackageBuf[i] = head[i];
    }

    int cntTemp = 0;
    for(int i = 0;i<rgbSize/MIN_TRANS_BYTE_NUM+1;i++)
    {
        if(i == rgbSize/MIN_TRANS_BYTE_NUM) //最后完成标志
        {
            rgbSubPackageBuf[3] = '!';
        }
        else
        {
            rgbSubPackageBuf[3] = i;
        }
        int j = 0;
        for (; j < MIN_TRANS_BYTE_NUM; j++)  
        {  
            rgbSubPackageBuf[j+4] = image_encode[cntTemp++]; 
            if(cntTemp == rgbSize) 
            {
                j++;
                break;
            }
        } 
        //分包发送
        while(isUdpUsage);
        SendData(rgbSubPackageBuf,j+4);
        //传输完成
        if(j<MIN_TRANS_BYTE_NUM)
        {
            break;
        }
    }
}
//float数组转uchar数组
void floatArr2Bytes(float* floatArr, unsigned int len, unsigned char* byteBufOut) {
    unsigned int pos = 0;
    unsigned char * temp = nullptr;
    int k = 0;
    for (int i = 0; i < len; i++) {
        temp = (unsigned char *)(&floatArr[i]);
        
        for (k = 0; k < 4; k++) {
            byteBufOut[pos++] = *temp++;
        }
    }
}
void depthImage_socket(Mat inImg,char head[3])
{
    //imshow("image_socket", inImg);//显示图片需要转成0～255灰度图
    //waitKey(1);
    if( inImg.empty() )
    {
      ROS_INFO("Camera image empty");
      return;//break;
    }
    Mat depth = inImg;
    int depthsize = depth.total()*depth.elemSize();
    // uchar depthSubPackageBufTemp[depthsize*4];
    // float depthValueBuf[depthsize];
    // for(int i = 0;i<depth.rows;i++)
    // {
    //     for(int j = 0;j<depth.cols;j++)
    //     {
    //         depthValueBuf[i*depth.cols+j] = depth.at<float>(i,j);
    //     }
    // }
    // floatArr2Bytes(depthValueBuf,depthsize,depthSubPackageBufTemp);
    depth = depth.reshape(0,1);
    ROS_INFO("depthSize: %d",depthsize);
    for(int i = 0;i<3;i++)
    {
        depthSubPackageBuf[i] = head[i];
    }
    int cntTemp = 0;
    for(int i = 0;i<depthsize/MIN_TRANS_BYTE_NUM+1;i++)
    {
        if(i == depthsize/MIN_TRANS_BYTE_NUM) //最后完成标志
        {
            depthSubPackageBuf[3] = '!';
        }
        else
        {
            depthSubPackageBuf[3] = i;
        }
        int j = 0;
        for (; j < MIN_TRANS_BYTE_NUM; j++)  
        {  
            depthSubPackageBuf[j+4] = depth.data[cntTemp++]; 
            // depthSubPackageBuf[j+4] = depthSubPackageBufTemp[cntTemp++]; 
            if(cntTemp == depthsize) 
            {
                j++;
                break;
            }
        } 
        //分包发送
        while(isUdpUsage);
        SendData(depthSubPackageBuf,j+4);
        //传输完成
        if(j<MIN_TRANS_BYTE_NUM)
        {
            break;
        }
    }
}

int rgbCallBackCnt = 0;
//图像回调
void rgbImageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    rgbCallBackCnt++;
    if(rgbCallBackCnt == 1)
    {
        cv_bridge::CvImagePtr cv_ptr;  
        try  
        {    
            cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);  
        }  
        catch (cv_bridge::Exception& e)  
        {  
            //if there is an error during conversion, display it  
            ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());  
            return;  
        }  
        rgbImage_socket(cv_ptr->image,"rgb");
        rgbCallBackCnt = 0;
    }
}
int depthCallBackCnt = 0;
void depthImageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    depthCallBackCnt++;
    if(depthCallBackCnt == 1)
    {
        cv_bridge::CvImagePtr cv_ptr;  
        try  
        {    
            cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::TYPE_32FC1);  //不同的深度值修改这里
        }  
        catch (cv_bridge::Exception& e)  
        {  
            //if there is an error during conversion, display it  
            ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());  
            return;  
        }  
        depthImage_socket(cv_ptr->image,"dep");
        depthCallBackCnt = 0;
    }
}

int velodyneCallbackCnt = 0;
void velodynePointCallback(const sensor_msgs::PointCloud2::ConstPtr &laserCloudIn)
{
    if(velodyneCallbackCnt>0){
        --velodyneCallbackCnt;
        return;
    }
    velodyneCallbackCnt=10;

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*laserCloudIn, *laserCloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloud, *laserCloud, indices);

    float verticalAngle, horizonAngle, range;
    int rowIdn, columnIdn, index;
    cv::Mat rangeMat = cv::Mat(16, 1800, CV_32FC1, cv::Scalar::all(-1.0));
    pcl::PointXYZI thisPoint;

    int cloudSize = laserCloud->points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
      thisPoint.x = laserCloud->points[i].x;
      thisPoint.y = laserCloud->points[i].y;
      thisPoint.z = laserCloud->points[i].z;

      verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
      rowIdn = (verticalAngle + 15.0 + 0.1) / 2.0;
      if (rowIdn < 0 || rowIdn >= 16)
        continue;

      horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI;
      columnIdn = (horizonAngle + 180.0 - 0.1) / 0.2;
      if (columnIdn >= 1800)
        columnIdn -= 1800;
      if (columnIdn < 0 || columnIdn >= 1800)
        continue;

      range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
      if (range < 0.3)
        continue;
      rangeMat.at<float>(rowIdn, columnIdn) = range;
    }

    depthImage_socket(rangeMat,"dep");

}

void mapCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &laserCloudIn)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*laserCloudIn,*laserCloud);
    int size = laserCloud->points.size();
    cv::Mat xyzMat = cv::Mat(1,size,CV_32FC3);
    
}

int main(int argc, char **argv)
{
    /************************Initiate ROS***********************/
    ros::init(argc, argv, "udpClientTrans"); //初始化节点
    ros::AsyncSpinner spinner(1); //开启线程
    spinner.start();
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    //订阅图像信息
    image_transport::Subscriber subRGB = it.subscribe("/usb_cam/image_raw", 1, rgbImageCallback);
    image_transport::Subscriber subDepth = it.subscribe("/depth/image", 1, depthImageCallback);
    ros::Subscriber subVelodynePoint = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",1,velodynePointCallback);
    ros::Subscriber subMapCloud = nh.subscribe<sensor_msgs::PointCloud2>("laser_surround_cloud",1,mapCloudCallback);
    /***********************************************************/

	//创建套接字
	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		cout<<"socket error"<<endl;
		return false;
	}
	bzero(&serveradr, sizeof(struct sockaddr_in));
	serveradr.sin_family = AF_INET;
	serveradr.sin_addr.s_addr = inet_addr("127.0.0.1");//inet_addr("192.168.1.250");
	serveradr.sin_port = htons(11000);

    while(ros::ok())
    {
        //起始时间	
        double start = ros::Time::now().toSec();
        //接收指令数据
        ros::spinOnce();

        // SendData((uchar*)"aaa",3);

        double sleep_time = LOOP_TIME - ( ros::Time::now().toSec() - start );
        if ( sleep_time > 0 )
        {
            ros::Duration ( sleep_time ).sleep();
            //ROS_INFO("Cost Time: %lf ms", (LOOP_TIME-sleep_time)*1000);
        }
        else
            ROS_WARN ( "control loop over time: %f ms", -sleep_time*1000 );
    }
    ros::waitForShutdown();
    return 0;
}