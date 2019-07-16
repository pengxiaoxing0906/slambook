//
// Created by pxx on 4/24/19.
//

#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>
namespace myslam {
    class VisualOdometry {
    public:
        typedef  shared_ptr<VisualOdometry>Ptr;
        enum VOState{INITIALIZING=-1,
                OK=0,
                LOST};


        VOState   state_; //current VO state
        Map::Ptr  map_; //map with all frames and map points
        Frame::Ptr ref_;  //reference frame
        Frame::Ptr curr_; //current frame

        cv::Ptr<cv::ORB>orb_; //orb detector and computer
        //vector<cv::Point3f>pts_3d_ref_; //3d points in reference frame
        vector<cv::KeyPoint>keypoints_curr_;//keypoints in current frame
        Mat descriptors_curr_;//desciptors in current frame
       // Mat descriptors_ref_; //descriptors in reference frame;

       cv::FlannBasedMatcher matcher_flann_; //flann matcher 以下三条新加的
       vector<MapPoint::Ptr> match_3dpts_; //matched 3d points
       vector<int> match_2dkp_index_; //matched 2d pixels(index of kp_curr)


        SE3 T_c_w_estimated_; //the estimated pose of current frame
        int num_inliers_;  //number of inlier features in icp
        int num_lost_; //number of lost times

        //parameters
        int num_of_features_;//number of features
        double scale_factor_;//scale in image pyramid 不知道干嘛用的
        int level_pyramid_;//number of pyramid levels 这个也不知道干嘛用的
        float match_ratio_;//ratio for selecting good matches
        int max_num_lost_; //max number of continuous lost times
        int min_inliers_; //minimum inliners

        double key_frame_min_rot;//minimal rotation of two key-frames
        double key_frame_min_trans; //minimal translation of two key-frames
        double map_point_erase_ratio_; //remove map point ratio

    public:
        VisualOdometry();
        ~VisualOdometry();

        bool addFrame(Frame::Ptr frame); //add a new frame

    protected:
        void extractKeyPoints();
        void computeDescriptors();
        void featureMatching();
        void poseEstimationPnP();
      //  void setRef3DPoints();
      void optimizeMap(); //以下三个新添的
      void addMapPoints();
      double getViewAngle(Frame::Ptr frame,MapPoint::Ptr point);

        void addKeyFrame();
        bool checkEstimatedPose();
        bool checkKeyFrame();




    };
}


#endif //MYSLAM_VISUAL_ODOMETRY_H
