#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "DBoW3/DBoW3.h"
#include <vector>
#include <string>

using namespace std;
using namespace cv;

int main(int argc,char**argv)
{
    //read image
    cout<<"reading images.."<<endl;
    vector<Mat>images;
    for(int i=0;i<10;i++)
    {
        Mat image;
        string path="./data/"+to_string(i+1)+".png";
        image=imread(path);
        images.push_back(image);
    }

    //提取所有的描述子
    cout<<"detecting ORB features..."<<endl;
    vector<Mat>descriptors;
    Ptr<Feature2D>detector=ORB::create();
    for(int i=0;i<10;i++)
    {
        vector<KeyPoint>keypoints;
        Mat descriptor;
        detector->detectAndCompute(images[i],Mat(),keypoints,descriptor);
        descriptors.push_back(descriptor);

    }

    //用描述子创建字典
    cout<<"creating vocabulary ..."<<endl;
    DBoW3::Vocabulary vocab;
    vocab.create(descriptors);
    cout<<"vocabulary info:"<<vocab<<endl;
    vocab.save("vocabulary.yml.gz");
    cout<<"done."<<endl;


    return 0;
}