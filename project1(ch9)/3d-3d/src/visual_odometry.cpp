/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

namespace myslam
{

    VisualOdometry::VisualOdometry() :
            state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 )
    {
        num_of_features_    = Config::get<int> ( "number_of_features" );
        scale_factor_       = Config::get<double> ( "scale_factor" );
        level_pyramid_      = Config::get<int> ( "level_pyramid" );
        match_ratio_        = Config::get<float> ( "match_ratio" );
        max_num_lost_       = Config::get<float> ( "max_num_lost" );
        min_inliers_        = Config::get<int> ( "min_inliers" );
        key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
        key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
        orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    }

    VisualOdometry::~VisualOdometry()
    {

    }

    bool VisualOdometry::addFrame ( Frame::Ptr frame )
    {
        switch ( state_ )
        {
            case INITIALIZING:
            {
                state_ = OK;
                curr_ = ref_ = frame;
                map_->insertKeyFrame ( frame );
                // extract features from first frame
                extractKeyPoints();
                computeDescriptors();
                // compute the 3d position of features in ref frame
                setRef3DPoints();
                break;
            }
            case OK:
            {
                curr_ = frame;
                extractKeyPoints();
                computeDescriptors();
                featureMatching();
                setCurr3DPoints();
                //poseEstimationPnP();
                poseEstimationSVD();
                if ( checkEstimatedPose() == true ) // a good estimation
                {
                    curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;  // T_c_w = T_c_r*T_r_w
                    ref_ = curr_;
                    //setRef3DPoints();
                    num_lost_ = 0;
                    if ( checkKeyFrame() == true ) // is a key-frame
                    {
                        addKeyFrame();
                    }
                }
                else // bad estimation due to various reasons
                {
                    num_lost_++;
                    if ( num_lost_ > max_num_lost_ )
                    {
                        state_ = LOST;
                    }
                    return false;
                }
                break;
            }
            case LOST:
            {
                cout<<"vo has lost."<<endl;
                break;
            }
        }

        return true;
    }

    void VisualOdometry::extractKeyPoints()
    {
        orb_->detect ( curr_->color_, keypoints_curr_ );
        // cout<<"keypoints_curr_: "<<keypoints_curr_.size()<<endl;
    }

    void VisualOdometry::computeDescriptors()
    {
        orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    }

    void VisualOdometry::featureMatching()
    {
        // match desp_ref and desp_curr, use OpenCV's brute force match
        vector<cv::DMatch> matches;
        cv::BFMatcher matcher ( cv::NORM_HAMMING );
        matcher.match ( descriptors_ref_, descriptors_curr_, matches );
        // select the best matches
        float min_dis = std::min_element (
                matches.begin(), matches.end(),
                [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
                {
                    return m1.distance < m2.distance;
                } )->distance;

        feature_matches_.clear();
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
            {
                feature_matches_.push_back(m);
            }
        }
        cout<<"good matches: "<<feature_matches_.size()<<endl;
    }

    void VisualOdometry::setRef3DPoints()
    {
        // select the features with depth measurements
        pts_3d_ref_.clear();
        descriptors_ref_ = Mat();
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            double d = ref_->FindDepth(keypoints_curr_[i]);
            if ( d > 0)
            {
                Vector3d p_cam = curr_->camera_->pixel2camera(
                        Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
                );
                pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
                descriptors_ref_.push_back(descriptors_curr_.row(i));
            }
        }
    }
    void VisualOdometry::setCurr3DPoints()
    {
        pts_3d_curr_.clear();
        descriptors_curr_ = Mat();
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            double d = curr_->FindDepth(keypoints_curr_[i]);
            if ( d > 0)
            {
                Vector3d p_cam = curr_->camera_->pixel2camera(
                        Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
                );
                pts_3d_curr_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
               // descriptors_ref_.push_back(descriptors_curr_.row(i));
            }
        }
    }

/*    void VisualOdometry::poseEstimationPnP()
    {
        // construct the 3d 2d observations
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        for ( cv::DMatch m:feature_matches_ )
        {
            pts3d.push_back( pts_3d_ref_[m.queryIdx] );
            pts2d.push_back( keypoints_curr_[m.trainIdx].pt );
        }
       // cout<<"pts3d: "<<pts3d.size()<<endl;
      //  cout<<"pts2d: "<<pts2d.size()<<endl;

        Mat K = ( cv::Mat_<double>(3,3)<<
                                       ref_->camera_->fx_, 0, ref_->camera_->cx_,
                0, ref_->camera_->fy_, ref_->camera_->cy_,
                0,0,1
        );
        Mat rvec, tvec, inliers;
        cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
        num_inliers_ = inliers.rows;
        cout<<"pnp inliers: "<<num_inliers_<<endl;
        T_c_r_estimated_ = SE3(
                SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
                Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
        );
    }
    */
    void VisualOdometry::poseEstimationSVD()
    {
        cv::Point3f p_ref_,p_curr_;
        int N= pts_3d_ref_.size();
        for(int i=0;i<N;i++)
       {
        p_ref_+=pts_3d_ref_[i];
        p_curr_+=pts_3d_curr_[i];

        }
        p_ref_/=N;
        p_curr_/=N;
        vector<cv::Point3f> q1(N),q2(N);
        for(int i=0;i<N;i++)
        {
            q1[i]=pts_3d_ref_[i]-p_ref_;
            q2[i]=pts_3d_curr_[i]-p_curr_;
        }
        //copmpute q1*q2^T
        Eigen::Matrix3d W=Eigen::Matrix3d::Zero();
        for(int i=0;i<N;i++)
        {
            W+=Eigen::Vector3d(q1[i].x,q1[i].y,q1[i].z)*Eigen::Vector3d(q2[i].x,q2[i].y,q2[i].z).transpose();
        }
        cout<<"W="<<W<<endl;

        //SVD on W
        Eigen::JacobiSVD<Eigen::Matrix3d>svd(W,Eigen::ComputeFullU|Eigen::ComputeFullV);
        Eigen::Matrix3d U=svd.matrixU();
        Eigen::Matrix3d V=svd.matrixV();
        cout<<"U="<<U<<endl;
        cout<<"V="<<V<<endl;

        Eigen::Matrix3d R_=U*(V.transpose());
        Eigen::Vector3d t_=Eigen::Vector3d(p_ref_.x,p_ref_.y,p_ref_.z)-R_*Eigen::Vector3d(p_curr_.x,p_curr_.y,p_curr_.z);

        //convert to cv::Mat
        Mat R1,t1;
        R1=(cv::Mat_<double>(3,3)<<R_(0,0),R_(0,1),R_(0,2),
                R_(1,0),R_(1,1),R_(1,2),
                R_(2,0),R_(2,1),R_(2,2));
        t1=(cv::Mat_<double>(3,1)<<t_(0,0),t_(1,0),t_(2,0));
        Mat R,t;
        R=R1.t();
        t=-R1.t()*t1;
        cout<<"R_inv="<<R.t()<<endl;
        cout<<"t_inv="<<-R.t()*t<<endl;

        T_c_r_estimated_ = SE3(
                SO3(R.at<double>(0,0), R.at<double>(1,0), R.at<double>(2,0)),
                Vector3d( t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0))
                );


    }

    bool VisualOdometry::checkEstimatedPose()
    {
        // check if the estimated pose is good
       //if ( num_inliers_ < min_inliers_ )
       // {
       //     cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
      //      return false;
     //   }
        // if the motion is too large, it is probably wrong
        Sophus::Vector6d d = T_c_r_estimated_.log();
        if ( d.norm() > 5.0 )
        {
            cout<<"reject because motion is too large: "<<d.norm()<<endl;
            return false;
        }
        return true;
    }

    bool VisualOdometry::checkKeyFrame()
    {
        Sophus::Vector6d d = T_c_r_estimated_.log();
        Vector3d trans = d.head<3>();
        Vector3d rot = d.tail<3>();
        if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
            return true;
        return false;
    }

    void VisualOdometry::addKeyFrame()
    {
        cout<<"adding a key-frame"<<endl;
        map_->insertKeyFrame ( curr_ );
    }

}