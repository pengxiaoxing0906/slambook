#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
using namespace std;
using namespace cv;

int main(int argc,char**argv)
{
   if(argc!=3)
   {
       cerr<<"有误啊"<<endl;
       return 1;
   }
   //读取两张图
   Mat img1=imread(argv[1],CV_LOAD_IMAGE_COLOR);
   Mat img2=imread(argv[2],CV_LOAD_IMAGE_COLOR);

   //初始化，定义关键点和描述子
   vector<KeyPoint>keypoints1,keypoints2;
   Mat descriptors1,descriptors2;
   Ptr<ORB>orb=ORB::create();

   //检测oriented FAST角点位置
   orb->detect(img1,keypoints1);
   orb->detect(img2,keypoints2);

   //计算BRIEF描述子
   orb->compute(img1,keypoints1,descriptors1);
   orb->compute(img2,keypoints2,descriptors2);

   //描绘出特征点
  // Mat outimg1;
   //drawKeypoints(img1,keypoints1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
   //imshow("ORB特征点",outimg1);

   //使用Hamming距离，BRIEF描述子进行匹配
   vector<DMatch>matches;
   BFMatcher matcher(NORM_HAMMING);
   matcher.match(descriptors1,descriptors2,matches);
   cout<<"Find total "<<matches.size()<<" matches."<<endl;

   //匹配点筛选
   double min_dist=1000,max_dist=0;
   for(int i=0;i<matches.size();i++)
   {
       double dist=matches[i].distance;
       if(dist<min_dist)min_dist=dist;
       if(dist>max_dist)max_dist=dist;
   }
   cout<<"min_dist: "<<min_dist<<endl;
   cout<<"max_dist: "<<max_dist<<endl;
   vector<DMatch>good_matches;
   for(int i=0;i<matches.size();i++)
   {
       if(matches[i].distance<=max(2*min_dist,30.0))
           good_matches.push_back(matches[i]);
   }

    vector< Point2f > pts1, pts2;
    for (size_t i=0; i<good_matches.size(); i++)
    {
        pts1.push_back(keypoints1[good_matches[i].queryIdx].pt);
        pts2.push_back(keypoints2[good_matches[i].trainIdx].pt);
    }

    // 请先计算基础矩阵并据此绘制出前10个匹配点对应的对极线，可以调用opencv函数
    // ----------- 开始你的代码 --------------//
    //相机内参
    Mat K=(Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
    //计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix=findFundamentalMat(pts1,pts2,CV_FM_8POINT);
    cout<<"fundamental_matrix is:"<<endl<<fundamental_matrix<<endl;
    vector<Vec3f>lines1,lines2;
    computeCorrespondEpilines(pts1,1,fundamental_matrix,lines1);
    computeCorrespondEpilines(pts2,2,fundamental_matrix,lines2);
    RNG rng(0);
    for(int i=0;i<10;i++)
    {
        const Scalar color(rng(256),rng(256),rng(256));
        circle(img1,pts1[i],5,color,2);
        circle(img2,pts2[i],5,color,2);

        Point pt1Left(0,-lines1[i][2]/lines1[i][1]);
        Point pt1Right(img1.cols,(-lines1[i][2]-lines1[i][0]*img1.cols)/lines1[i][1]);
        line(img2,pt1Left,pt1Right,color,1);

        Point pt2Left(0,-lines2[i][2]/lines2[i][1]);//在视图2中把关键点用圆圈画出来，然后再绘制在对应点处的外极线
        Point pt2Right(img2.cols,(-lines2[i][2]-lines1[i][0]*img2.cols)/lines2[i][1]);
        //绘制外极线的时候，选择两个点，一个是x=0处的点，一个是x为图片宽度处
        line(img1,pt2Left,pt2Right,color,1);

    }

    // ----------- 结束你的代码 --------------//
    imshow("epiline1", img2);
    imshow("epiline2", img1);
   waitKey(0);
    return 0;
}