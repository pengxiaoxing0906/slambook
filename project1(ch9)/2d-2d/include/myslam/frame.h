//
// Created by pxx on 4/24/19.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

//#include <bits/shared_ptr.h>
#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam {
    class MapPoint;//[5]加这一句在这里干嘛？
    class Frame {
    public:
        typedef std::shared_ptr<Frame>Ptr;
        unsigned long   id_; //id of this frame
        double          time_stamp_; //when it is recorded
        SE3             T_c_w_; //transform from world to camera
        Camera::Ptr     camera_; //Pinhole RGBD Camera model
        Mat             color_,depth_; //color and depth image

    public:
        Frame();
        Frame(long id,double time_stamp=0,SE3 T_c_w=SE3(),Camera::Ptr camera= nullptr,Mat color=Mat(),Mat depth=Mat());
        ~Frame();

        //factory function
        static Frame::Ptr createFrame();

        //find the depth in depth map
        double FindDepth(const cv::KeyPoint&kp);

        //get camera center
        Vector3d getCamcenter()const;


        //check if a point is in the frame
        bool isInFrame(const Vector3d& pt_world);

    };
}


#endif //MYSLAM_FRAME_H
