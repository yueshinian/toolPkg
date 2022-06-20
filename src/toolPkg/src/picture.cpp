#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    //照片result1.jpg改变像素（尺寸大小）
    Mat src = imread("/home/yz/catkin_ws/catkin_ym/src/toolPkg/src/result1.jpg");
    if(src.empty()){
        cout<<"error"<<endl;
        return 0;
    }
    cout << src.size() << endl;
    Size srcSize = Size(350, 530);  //填入任意指定尺寸
    resize(src, src, srcSize);
    cout << src.size() << endl;
    //imshow("压缩图", src);
    imwrite("/home/yz/catkin_ws/catkin_ym/src/toolPkg/src/result2.jpg", src);  //保存图片
    waitKey(0);
    return 0;
    return 0;
}