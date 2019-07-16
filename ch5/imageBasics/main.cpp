#include <iostream>
#include<chrono>
using namespace std;


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc,char**argv)
{
    //read image
    cv::Mat image1,image2; //创建一个空图像
    string imagepath = "/home/pxx/slambook/ch5/imageBasics/hcy.jpg";
    image1=cv::imread(imagepath,1); //读取输入图像
    image2=cv::imread(imagepath,0);
    if(image1.empty())
    {
        cout<<"error"<<endl;
    }


    //show image
    cout<<"the width of image "<<image1.cols<<" and the height of image "<<image1.rows<<" and the channels are"<<image1.channels()<<endl;
    cv::namedWindow("image");
    cv::namedWindow("image_gray");
    cv::imshow("image",image1);
    cv::imshow("image_gray",image2);
    cv::waitKey(0);

    //遍历图像
    for(size_t y=0;y<image1.rows;y++) {
        for (size_t x = 0; x < image1.cols; x++) {
            unsigned char *row_ptr = image1.ptr<unsigned char>(y); //row_ptr是第y行的头指针
            unsigned char *data_ptr = &row_ptr[x * image1.channels()];//data_ptr指向待访问的像素数据
            //输出该像素的每个通道，如果是灰度图就只有一个通道
            for (int c = 0; c != image1.channels(); c++) {
                unsigned char data = data_ptr[c];
            }
        }
    }


    //关于cv::Mat的拷贝 直接赋值并不会复制数据
    cv::Mat image_another=image1;
    //修改image_another会导致image1变化
    image_another(cv::Rect(0,0,100,100)).setTo(0);//将左上角100*100的块置零
    cv::imshow("image",image1);
    cv::waitKey(0);

    //使用clone函数来复制数据
    cv::Mat image_clone=image1.clone();
    image_clone(cv::Rect(0,0,100,100)).setTo(255);
    cv::imshow("image",image1);
    cv::imshow("image_clone",image_clone);
    cv::waitKey(0);


    return 0;
}