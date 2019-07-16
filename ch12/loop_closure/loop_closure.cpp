#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <string>
#include "DBoW3/DBoW3.h"

using namespace std;
using namespace cv;

int main(int argc,char**argv)
{
    cout<<"读取字典"<<endl;
    DBoW3::Vocabulary vocab("./vocabulary.yml.gz");
    if(vocab.empty())
    {
        cerr<<"字典不存在"<<endl;
        return 1;
    }

    cout<<"读取图片"<<endl;
    vector<Mat>images;
    for(int i=0;i<10;i++)
    {
        string path="./data/"+to_string(i+1)+".png";
        Mat image;
        image=imread(path);
        images.push_back(image);
    }

    cout<<"提取ORB特征点"<<endl;
    vector<Mat>descriptors;
    Ptr<Feature2D>detector=ORB::create();
    for(Mat& Img:images)
    {
        vector<KeyPoint>keypoints;
        Mat descriptor;
        detector->detectAndCompute(Img,Mat(),keypoints,descriptor);
        descriptors.push_back(descriptor);
    }

    cout<<"comparing image with image"<<endl;
    for(int i=0;i<images.size();i++)
    {
        DBoW3::BowVector v1;
        vocab.transform(descriptors[i],v1); //Transforms a set of descriptores into a bow vector
        for(int j=i;j<images.size();j++)
        {
            DBoW3::BowVector v2;
            vocab.transform(descriptors[j],v2);
            double score=vocab.score(v1,v2);
            cout<<"image "<<i<<" vs image "<<j<<" : "<<score<<endl;
        }
        cout<<endl;
    }

    cout<<"comparing image with database"<<endl;
    DBoW3::Database db(vocab, false,0);
    for(int i=0;i<descriptors.size();i++)
        db.add(descriptors[i]);
    cout<<"database info: "<<db<<endl;
    for(int i=0;i<descriptors.size();i++)
    {
        DBoW3::QueryResults ret;
        db.query(descriptors[i],ret,4); //max result =4
        cout<<"searching for image "<<i<<" returns "<<ret<<endl<<endl;

    }

    cout<<"done."<<endl;
}

