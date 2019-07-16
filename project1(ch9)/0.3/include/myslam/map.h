//
// Created by pxx on 4/24/19.
//

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {
    class Map {
    public:
        typedef shared_ptr<Map>Ptr;
        unordered_map<unsigned  long,MapPoint::Ptr>map_points_; //all landmarks [9]容器的用法
        unordered_map<unsigned long,Frame::Ptr>keyframes_; //all key_frames
        Map();
        void insertKeyFrame(Frame::Ptr frame);
        void insertMapPoint(MapPoint::Ptr map_point);


    };
}


#endif //MYSLAM_MAP_H
