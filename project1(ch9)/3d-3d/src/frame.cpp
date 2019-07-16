//
// Created by pxx on 4/24/19.
//

#include "../include/myslam/frame.h"
#include "myslam/camera.h"
#include "myslam/common_include.h"
namespace myslam {
    Frame::Frame():id_(-1),time_stamp_(-1),camera_(nullptr)
    {

    }

    Frame::Frame(long id, double time_stamp , SE3 T_c_w , Camera::Ptr camera , Mat color ,
          Mat depth ):id_(id),time_stamp_(time_stamp),T_c_w_(T_c_w),camera_(camera),color_(color),depth_(depth)
{

}



    Frame::~Frame()
    {

    }

//factory function
   Frame::Ptr Frame::createFrame()
    {
        //这里注意下，由factory_id++一个数去构造Frame对象时，调用的是默认构造函数，由于默认构造函数全都有默认值，所以就是按坑填，先填第一个id_，
        //所以也就是相当于创建了一个只有ID号的空白帧
        // [6]不理解这句的语法 感觉很奇怪
        static long factory_id=0;
        return Frame::Ptr(new Frame(factory_id++));
    }

//find the depth in depth map
    double Frame::FindDepth(const cv::KeyPoint &kp)
    {
        int x=cvRound(kp.pt.x);
        int y=cvRound(kp.pt.y);
        ushort d=depth_.ptr<ushort>(y)[x];
        if(d!=0)
        {
            return double(d)/camera_->depth_scale_;
        }
        else
        {
            //check nearby points
            int dx[4]={-1,0,1,0};
            int dy[4]={0,-1,0,1};
            for(int i=0;i<4;i++)
            {
                d=depth_.ptr<ushort >(y+dy[i])[x+dx[i]];
                if(d!=0)
                {
                    return  double(d)/camera_->depth_scale_;
                }
            }
        }
        return  -1.0;

    }

//get camera center
    Vector3d Frame::getCamcenter() const
    {
        //[7]不理解为啥这样就是相机中心的位置
        return T_c_w_.inverse().translation();

    }


//check if a point is in the frame
    bool Frame::isInFrame(const Vector3d &pt_world)
    {
        Vector3d p_cam=camera_->world2camera(pt_world,T_c_w_);
        if(p_cam(2,0)<0)
            return false;
        Vector2d pixel=camera_->camera2pixel(p_cam);
       // Vector2d pixel=camera_->world2pixel(pt_world,T_c_w_);
       if(pixel(0,0)>0&&pixel(1,0)>0&&pixel(0,0)<color_.cols&&pixel(1,0)<color_.rows) //pixel(0,0)代表u,pixel(1,0)代表v
       {
           return true;
       }


    }
}
