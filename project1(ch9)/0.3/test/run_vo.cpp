#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/opencv.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

int main(int argc,char**argv)
{
    cout<<"argc "<<argc<<endl;
    cout<<"argv[1]:"<<argv[1]<<endl;
    if(argc!=2)
    {
        cout<<"usage:run_vo parameter_file "<<endl;
        return 1;
    }


    myslam::Config::setParameterFile(argv[1]);
 //   cout<<"argv[1]"<<argv[1]<<endl;
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);

    string dataset_dir=myslam::Config::get<string>("dataset_dir");
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin(dataset_dir+"/associate.txt");
    if(!fin)
    {
        cout<<"please generate the associate file called associate.txt"<<endl;
        return 1;
    }
    vector<string>rgb_files,depth_files;
    vector<double>rgb_times,depth_times;
    while(!fin.eof())
    {
        string rgb_time,rgb_file,depth_time,depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back(atof(rgb_time.c_str()));//atof()把字符串转成浮点数 c_str()返回一个指向正规C字符串的指针，内容与本string相同
        depth_times.push_back(atof(depth_time.c_str()));
        rgb_files.push_back(dataset_dir+"/"+rgb_file);
        depth_files.push_back(dataset_dir+"/"+depth_file);
        if(fin.good()== false)
            break;
    }
    myslam::Camera::Ptr camera(new myslam::Camera);

    //visualization
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0),camera_coor(0.5); // 构造参数是坐标系长度，也就是可视窗里的锥形小坐标系的长度，下面对坐标系部件进行设置
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH,2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH,1.0);
    vis.showWidget("world",world_coor);
    vis.showWidget("camera",camera_coor);
    //设置视角。这步是非必要步骤，进行设置有利于观察，
    //不设置也会有默认视角，就是可能比较别扭。而且开始后拖动鼠标，也可以改变观察视角。
    //构建三个3D点,这里其实就是构造makeCameraPose()函数需要的三个向量：
    //相机位置坐标、相机焦点坐标、相机y轴朝向
    //蓝色-Z，红色-X，绿色-Y
    cv::Point3d cam_pos(0,-1.0,-1.0),cam_focal_point(0,0,0),cam_y_dir(0,1,0);
    cv::Affine3d cam_pose=cv::viz::makeCameraPose(cam_pos,cam_focal_point,cam_y_dir);
    vis.setViewerPose(cam_pose);

    cout<<"read total "<<rgb_files.size()<<" entries"<<endl;
    for(int i=0;i<rgb_files.size();i++)
    {
        Mat color=cv::imread(rgb_files[i]);
        Mat depth=cv::imread(depth_files[i]);
        if(color.data== nullptr||depth.data== nullptr)
        {
            break;
        }
        myslam::Frame::Ptr pFrame=myslam::Frame::createFrame();
        pFrame->camera_=camera;
        pFrame->color_=color;
        pFrame->depth_=depth;
        pFrame->time_stamp_=rgb_times[i];

        boost::timer timer;
        vo->addFrame(pFrame);
        cout<<"VO costs time: "<<timer.elapsed()<<endl;
        if(vo->state_==myslam::VisualOdometry::LOST)
        {
            break;
        }
        SE3 Tcw=pFrame->T_c_w_.inverse();

        //show  the map and the camera pose
        cv::Affine3d M(
                cv::Affine3d::Mat3(
                        Tcw.rotation_matrix()(0,0),Tcw.rotation_matrix()(0,1),Tcw.rotation_matrix()(0,2),
                        Tcw.rotation_matrix()(1,0),Tcw.rotation_matrix()(1,1),Tcw.rotation_matrix()(1,2),
                        Tcw.rotation_matrix()(2,0),Tcw.rotation_matrix()(2,1),Tcw.rotation_matrix()(2,2)
                        ),
                        cv::Affine3d::Vec3(
                                Tcw.translation()(0,0),Tcw.translation()(1,0),Tcw.translation()(2,0)
                                )
                );
        cv::imshow("image",color);
        cv::waitKey(1);
        vis.setWidgetPose("camera",M);
        vis.spinOnce(1,false);
    }


    return 0;


}