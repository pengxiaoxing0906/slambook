#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc,char**argv)
{
    ifstream fin("./data/pose.txt");
    if (!fin)
    {
        cerr<<"cannot find pose file"<<endl;
        return 1;
    }

    vector<cv::Mat>colorImgs,depthImgs; //彩色图和深度图
    vector<Eigen::Isometry3d>poses; //相机位姿

    for ( int i=0; i<5; i++ )
    {
           boost::format fmt( "./data/%s/%d.%s" ); //图像文件格式
            Mat colorImg,depthImg;
           colorImg=cv::imread((fmt%"color"%(i+1)%"png").str() );
            if(!colorImg.data)
            {
                cout<<"aaa"<<endl;
                return 1;
            }
            colorImgs.push_back(colorImg );

           depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像

            double data[7] = {0};
            for ( int i=0; i<7; i++ )
            {
                fin>>data[i];
            }
            Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
            Eigen::Isometry3d T(q);
            T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
            poses.push_back( T );

    }


    //准备参数
    //相机内参
    double cx=325.5;
    double cy=253.5;
    double fx=518.0;
    double fy=519.0;
    double depthScale=1000.0;

    cout<<"正在将图像转换成点云..."<<endl;

    //定义点云使用的格式，这里用XYZRGB
    typedef  pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT>PointCloud;

    //新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    for(int i=0;i<5;i++)
    {
        PointCloud::Ptr current(new PointCloud);
        cout<<"转换图像中:"<<i+1<<endl;
        Mat color=colorImgs[i];
        Mat depth=depthImgs[i];
        Isometry3d T=poses[i];
        for(int v=0;v<color.rows;v++)
        {
            for(int u=0;u<color.cols;u++)
            {
                unsigned int d= depth.ptr<unsigned short>(v)[u];
                if (d == 0)continue;
                if (d >= 7000)continue;
                Vector3d point;
                point[2] = double(d) / depthScale;
                point[1]=(v-cy)*point[2]/fy;
                point[0]=(u-cx)*point[2]/fx;
                Vector3d pointWorld =T*point; //T指相机坐标系下的点转换到世界坐标系下


                 PointT p;
                 p.x=pointWorld[0];
                 p.y=pointWorld[1];
                 p.z=pointWorld[2];
                 p.b=color.data[v*color.step+u*color.channels()];
                 p.g=color.data[v*color.step+u*color.channels()+1];
                 p.r=color.data[v*color.step+u*color.channels()+2];
                 current->points.push_back(p);
            }
        }

        //跳出上两个for循环得到的是一张图的点云图　对每一张点云图都做剔除离群点的处理
        //depth filter and statistical removal
        PointCloud::Ptr tmp(new PointCloud);
        pcl::StatisticalOutlierRemoval<PointT>statistical_filter;
        statistical_filter.setMeanK(50); //设置在近领域搜索点数为５０
        statistical_filter.setStddevMulThresh(1.0);//假设领域的点为高斯分布在均值+stMul*方差　stMul=1.0
        statistical_filter.setInputCloud(current); //设置输入的点云
        statistical_filter.filter(*tmp);//滤波处理之后存入tmp
        (*pointCloud)+=*tmp;


    }

    pointCloud->is_dense= false;
    cout<<"点云共有"<<pointCloud->size()<<"个点。"<<endl;


    // voxel filter
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );       // resolution
    PointCloud::Ptr tmp ( new PointCloud );
    voxel_filter.setInputCloud( pointCloud );
    voxel_filter.filter( *tmp );
    tmp->swap( *pointCloud );

    cout<<"滤波之后，点云共有"<<pointCloud->size()<<"个点."<<endl;


    pcl::io::savePCDFileBinary("map.pcd",*pointCloud);



    return 0;
}
