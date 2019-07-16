#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <octomap/octomap.h>

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc,char**argv)
{
    vector<Mat>colorImgs,depthImgs;
    vector<Isometry3d>poses;

    ifstream fin("./data/pose.txt");
    if(!fin)
    {
        cout<<"文件打开失败！请在正确路径下打开文件"<<endl;
        return 1;
    }
    for(int i=0;i<5;i++)
    {
        boost::format fmt("./data/%s/%d.%s");//图像文件格式
        colorImgs.push_back(imread((fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back(imread((fmt%"depth"%(i+1)%"pgm").str(),-1)); //表示读入原始图像

        double data[7]={0};
        for(int i=0;i<7;i++)
        {
            fin>>data[i];

        }
        Quaterniond q(data[6],data[3],data[4],data[5]);
        Vector3d t(data[0],data[1],data[2]);
        Isometry3d T(q);
        T.pretranslate(t);
        poses.push_back(T);


    }

    // 相机内参
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;

    cout<<"正在将图像转换为octomap..."<<endl;

    octomap::OcTree tree(0.05); //参数为分辨率
    for(int i=0;i<5;i++)
    {
        cout<<"图像转换中"<<i+1<<endl;
        Mat color=colorImgs[i];
        Mat depth=depthImgs[i];
        Isometry3d T=poses[i];

        octomap::Pointcloud cloud; //the point cloud in octomap
        for(int v=0;v<color.rows;v++)
        {
            for(int u=0;u<color.cols;u++)
            {
                unsigned int d=depth.ptr<unsigned short >(v)[u];//深度值
                if(d==0)continue;
                if(d>=7000)continue;
                Vector3d point;
                point[2]=d/depthScale;
                point[1]=(v-cy)*point[2]/fy;
                point[0]=(u-cy)*point[2]/fx;
                Vector3d pointWorld=T*point;
                cloud.push_back(pointWorld[0],pointWorld[1],pointWorld[2]);
            }
        }

        //将点云存入八叉树地图，给定原点，这样可以计算投射线
        tree.insertPointCloud(cloud,octomap::point3d(T(0,3),T(1,3),T(2,3)));
    }

    //更新中间节点的占据信息并写入磁盘
    tree.updateInnerOccupancy();
    cout<<"saving octomap ..."<<endl;
    tree.writeBinary("octomap.bt");
    return 0;



}
