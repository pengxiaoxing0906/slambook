/****************************
 * 题目：相机视场角比较小（比如手机摄像头）时，一般可以近似为针孔相机成像，三维世界中的直线成像也是直线。
 * 但是很多时候需要用到广角甚至鱼眼相机，此时会产生畸变，三维世界中的直线在图像里会弯曲。因此，需要做去畸变。
 * 给定一张广角畸变图像，以及相机的内参，请完成图像去畸变过程
 *
* 本程序学习目标：
 * 掌握图像去畸变原理
 *
 * 时间：2018.10
****************************/

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc,char **argv)
{

    //准备参数
    double k1 = -0.28340811, k2 = 0.07395907;  // 畸变参数
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375; //相机内参

   Mat srcimage=imread("/home/pxx/Documents/slambook/ch5/undistort-image/test.png");
   Mat grayimage;
   cvtColor(srcimage,grayimage,CV_BGR2GRAY);
    int rows=grayimage.rows;
    int cols=grayimage.cols;
   Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图
    cv::imshow("image distorted", grayimage);

   //去畸变过程
   for(int v=0;v<rows;v++)
   {
       for(int u=0;u<cols;u++)
       {
           double u_distorted = 0, v_distorted = 0;
           double x = (u - cx) / fx;   //此时的x,y是正常图像的坐标
           double y = (v - cy) / fy;
           double r = x * x + y * y;

           double x_ = x * (1 + k1 * r + k2 * r);//此时x_,y_是畸变后的图像的坐标
           double y_ = y * (1 + k1 * r + k2 * r);

           u_distorted = fx * x_ + cx;
           v_distorted = fy * y_ + cy;
           if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows)
           {
               image_undistort.at<uchar>(v, u) = grayimage.at<uchar>((int) v_distorted, (int) u_distorted);
           }
           else
               image_undistort.at<uchar>(v, u) = 0;

       }

   }
    cv::imshow("image undistorted", image_undistort);
    cv::waitKey(0);

    return 0;
}