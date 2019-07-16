//
// Created by pxx on 4/24/19.
//

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H
#include "myslam/common_include.h"
namespace myslam {
    class Frame;
    class MapPoint {
    public:
        typedef shared_ptr<MapPoint>Ptr;
        unsigned  long id_; //ID
        Vector3d pos_;  //position in world
        Vector3d norm_; //Normal of viewing direction [8]其实没搞明白这变量干嘛使的
        Mat   descriptor_; //Descriptor for matching
        int observed_times_;//being observed by feature matching algo;
        int matched_times_; //being an inliner in pose estimation

        MapPoint();
        MapPoint(long id,Vector3d position,Vector3d norm);

        //factory function
        static  MapPoint::Ptr createMapPoint();



    };
}



#endif //MYSLAM_MAPPOINT_H
