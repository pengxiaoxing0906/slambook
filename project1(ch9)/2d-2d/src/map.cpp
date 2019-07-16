//
// Created by pxx on 4/24/19.
//

#include "../include/myslam/map.h"
namespace myslam {
    Map::Map() {}

   /* void Map::insertKeyFrame(Frame::Ptr frame)
    {
        cout<<"key frame size = "<<keyframes_.size()<<endl;
        if(keyframes_.find(frame->id_)==keyframes_.end())
        {
            keyframes_.insert(make_pair(frame->id_,frame)); //[10]make_pair的用法 if else 写的这个条件代表啥意思
        }
        else
        {
            keyframes_[frame->id_]=frame;
        }
    }
    */
    void Map::insertKeyFrame ( Frame::Ptr frame )
    {
        cout<<"Key frame size = "<<keyframes_.size()<<endl;
        if ( keyframes_.find(frame->id_) == keyframes_.end() )
        {
            keyframes_.insert( make_pair(frame->id_, frame) );
        }
        else
        {
            keyframes_[ frame->id_ ] = frame;
        }
    }

    void Map::insertMapPoint(MapPoint::Ptr map_point)
    {
        if(map_points_.find(map_point->id_)==map_points_.end())
        {
            map_points_.insert(make_pair(map_point->id_,map_point));
        }
        else
        {
            map_points_[map_point->id_]=map_point;
        }
    }
}
