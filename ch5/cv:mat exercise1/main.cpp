#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

//测试函数，它创建一个函数
cv::Mat functio(){
    //创建图像
    cv::Mat ima(500,500,CV_8U,50);
    //返回图像
    return ima;
}

int main() {
   //定义图像窗口
  //cv::namedWindow("image 1");
  //cv::namedWindow("image 2");
  //cv::namedWindow("image 3");
  //cv::namedWindow("image 4");
 // cv::namedWindow("image 5");
 //   cv::namedWindow("image");


   //创建一个240行×320列的新图像
   cv::Mat image1(240,320,CV_8U,100);
   cv::imshow("image",image1); //显示函数
   cv::waitKey(0); //等待按键

   //重新分配一个新的图像
   image1.create(200,200,CV_8U);
   image1=200;
  cv::imshow("image",image1); //显示图像
   cv::waitKey(0); //等待按键

   //创建一个红色的图像
   //通道次序为BGR
   cv::Mat image2(240,320,CV_8UC3,cv::Scalar(0,0,255));
  cv::imshow("image",image2);
   cv::waitKey(0);

   //读入一个图像
   cv::Mat image3=cv::imread("/home/pxx/slambook/ch5/cv:mat exercise1/hcy.jpg");
   if(image3.empty())   //错误处理
   {
       cout<<"no input"<<endl;
   }


   //所有这些图像指向同一个数据块
   cv::Mat image4(image3);
   image1=image3;

   //这些图像是源图像的副本图像
   image3.copyTo(image2);
   cv::Mat image5 =image3.clone();


   //转换图像用来测试
   cv::flip(image3,image3,1);

   //检查哪些图像在处理过程中受到了影响
   cv::imshow("image3",image3);
    cv::imshow("image1",image1);
    cv::imshow("image2",image2);
    cv::imshow("image4",image4);
    cv::imshow("image5",image5);
    cv::waitKey(0);


    //从函数中获取一个灰度图像
    cv::Mat gray= functio();
    cv::imshow("image",gray);
    cv::waitKey(0);


    //作为灰度图像读入
    image1=cv::imread("/home/pxx/slambook/ch5/cv:mat exercise1/hcy.jpg",CV_LOAD_IMAGE_GRAYSCALE);
    image1.convertTo(image2,CV_32F,1/255.0,0.0);
    cv::imshow("image",image2);
    cv::waitKey(0);



    return 0;
}