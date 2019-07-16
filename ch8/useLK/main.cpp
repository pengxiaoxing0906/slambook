#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
using namespace std;
using namespace cv;

int main(int argc,char**argv)
{
    if(argc!=2)
    {
        cout<<"usage:useLK path_to_dataset"<<endl;
        return 1;
    }
    string path_to_dataset=argv[1];
    string associate_file=path_to_dataset+"/associate.txt";
    ifstream fin(associate_file);
    if ( !fin )
    {
        cerr<<"I cann't find associate.txt!"<<endl;
        return 1;
    }
    string rgb_file,depth_file,time_rgb,time_depth;
    list< cv::Point2f > keypoints;      // 因为要删除跟踪失败的点，使用list
  //  list<cv::Point2f>keypoints;//因为要删除跟踪失败的点，使用list
    Mat color,depth,last_color;
    for(int index=0;index<100;index++)
    {
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
        color=imread(path_to_dataset+"/"+rgb_file);
        depth=imread(path_to_dataset+"/"+depth_file,-1);
        if(index==0)
        {
            //对第一帧提取FAST特征点，然后对第一帧图像提取的特征进行追踪
            vector<KeyPoint>kps;
            Ptr<FastFeatureDetector>detector=cv::FastFeatureDetector::create();
            detector->detect(color,kps);
            for(auto kp:kps)
            {
                keypoints.push_back(kp.pt);
            }
            last_color=color;
            continue;

        }
        if(color.data== nullptr||depth.data== nullptr)
            continue;
        //对其他帧用LK跟踪特征点
        vector<Point2f>next_keypoints;
        vector<Point2f>prev_keypoints;
        for(auto kp:keypoints)
        {
            prev_keypoints.push_back(kp);
        }
        vector<unsigned char>status;
        vector<float>error;
        chrono::steady_clock::time_point t1=chrono::steady_clock::now();
        calcOpticalFlowPyrLK(last_color,color,prev_keypoints,next_keypoints,status,error);//用LK光流法得到新一帧更新后的特征点位置
        chrono::steady_clock::time_point t2=chrono::steady_clock::now();
        chrono::duration<double>time_used=chrono::duration_cast<chrono::duration<double>>(t2-t1);
        cout<<"LK Flow use time: "<<time_used.count()<<" seconds."<<endl;

        //把跟丢的点删掉
        int i=0;
        for(auto iter=keypoints.begin();iter!=keypoints.end();i++)
        {
            if(status[i]==0)
            {
                iter=keypoints.erase(iter);
                continue;
            }
            *iter=next_keypoints[i];
            iter++;
        }
        cout<<"tracked keypoints: "<<keypoints.size()<<endl;
        if(keypoints.size()==0)
        {
            cout<<"all keypoints are lost."<<endl;
            break;
        }
        //画出keypoints
        Mat img_show=color.clone();
        for(auto kp:keypoints)
        {
            circle(img_show,kp,10,cv::Scalar(0,240,0),1);
            imshow("corners",img_show);

        }
        waitKey(0);
        last_color=color;
    }
    return 0;
}