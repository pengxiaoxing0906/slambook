#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;
void pose_estimation_2d2d(std::vector<KeyPoint>keypoints_1,
        std::vector<KeyPoint>keypoints_2,
        std::vector<DMatch>matches,
        Mat &R,Mat&t);
void find_feature_matches(const Mat &img1,const Mat&img2,
        std::vector<KeyPoint>&keypoints1,
        std::vector<KeyPoint>&keypoints2,
        std::vector<DMatch>&matches);
Point2d pixel2cam(const Point2d&p,const Mat&K);//像素坐标转相机归一化坐标
void triangulation( const vector<KeyPoint>&keypoints_1,
        const vector<KeyPoint>&keypoints_2,
       const std::vector<DMatch>&matches ,
       const Mat&R,const Mat&t,
       vector<Point3d>points);
int main(int argc,char**argv)
{
    //读取图像
    Mat img_1=imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img_2=imread(argv[2],CV_LOAD_IMAGE_COLOR);
    vector<KeyPoint>keypoints_1,keypoints_2;
    vector<DMatch>matches;
    find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
    cout<<"共找到了"<<matches.size()<<"组匹配点"<<endl;

    //估计两张图像之间的运动
    Mat R,t;
    pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);


    //验证E=t^R
    Mat t_x=(Mat_<double>(3,3)<<
            0,-t.at<double>(2,0),t.at<double>(1,0),
             t.at<double>(2,0),0,-t.at<double>(0,0),
             -t.at<double>(1,0),t.at<double>(0,0),0 );
    cout<<"t^R= "<<endl<<t_x*R<<endl;

    //验证对极约束
    Mat K=(Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
    for(DMatch m:matches)//对匹配上的特征点进行遍历
    {
        Point2d pt1=pixel2cam(keypoints_1[m.queryIdx].pt,K);//像素坐标转换为相机归一化坐标
        Mat y1=(Mat_<double>(3,1)<<pt1.x,pt1.y,1); //转换为齐次坐标
        Point2d pt2=pixel2cam(keypoints_2[m.trainIdx].pt,K);
        Mat y2=(Mat_<double>(3,1)<<pt2.x,pt2.y,1);
        Mat d=y2.t()*t_x*R*y1;
        cout<<"epiploar constraint = "<<d<<endl;
    }

    //三角化  可以获得3D的点
    vector<Point3d>points;
    triangulation(keypoints_1,keypoints_2,matches,R,t,points);

    //验证三角化与特征点的重投影关系
    for(int i=0;i<matches.size();i++)
    {
      Point2d  pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, K);
        cout<<points[i].x<<points[i].y<<points[i].z<<endl;
      Point2d pt1_cam_3d(points[i].x/points[i].z,points[i].y/points[i].z);



        cout << "point in  the first camera frame:" << pt1_cam << endl;
        cout << "point projected from 3D "<<pt1_cam_3d<<",d is "<<points[i].z<< endl;

        //第2张图
        Point2d  pt2_cam = pixel2cam(keypoints_1[matches[i].trainIdx].pt, K);
        Mat pt2_trans=R*(Mat_<double>(3,1)<<points[i].x,points[i].y,points[i].z)+t;
        pt2_trans/=pt2_trans.at<double>(2,0);
        cout << "point in  the second camera frame:" << pt2_cam<< endl;
        cout << "point projected from 3D " <<pt2_trans.t()<< endl;
    }


    return 0;
}
void pose_estimation_2d2d(std::vector<KeyPoint>keypoints_1,std::vector<KeyPoint>keypoints_2,std::vector<DMatch>matches,Mat &R,Mat&t)
{
    //[1]给相机内参
   Mat K=(Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
    //【2】对齐匹配的点对,并用.pt转化为像素坐标.把匹配点转换为简单的Point2f形式
   vector<Point2f>points1;
   vector<Point2f>points2;
   for(int i=0;i<(int)matches.size();i++)
   {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
    //[3]计算基础矩阵
   Mat fundamental_matrix;
    fundamental_matrix=findFundamentalMat(points1,points2,CV_FM_8POINT);
    cout<<"fundamental_matrix is "<<endl<<fundamental_matrix<<endl;

    //[4]计算本质矩阵
    Point2d principal_point(325.1,249.7);//相机光心
    double focal_length=521;//相机焦距
    Mat essential_matrix;
    essential_matrix=findEssentialMat(points1,points2,focal_length,principal_point);
    cout<<"essential matrix is "<<endl<<essential_matrix<<endl;

    //[5]计算单应矩阵
    Mat homography_matrix;
    homography_matrix=findHomography(points1,points2,RANSAC,3);
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //[6]从本质矩阵中恢复旋转和平移信息
    recoverPose(essential_matrix,points1,points2,R,t,focal_length,principal_point);
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;

}
void find_feature_matches(const Mat &img1,const Mat&img2,std::vector<KeyPoint>&keypoints1,std::vector<KeyPoint>&keypoints2,std::vector<DMatch>&matches)
{
    //初始化
    Mat descriptors_1,descriptors_2;
    Ptr<FeatureDetector>detector=ORB::create();
    Ptr<DescriptorExtractor>descriptor=ORB::create();
    Ptr<DescriptorMatcher>matcher=DescriptorMatcher::create("BruteForce-Hamming");
    //【1】检测oriented FAST角点位置
    detector->detect(img1,keypoints1);
    detector->detect(img2,keypoints2);
    //[2]计算BRIEF描述子
    descriptor->compute(img1,keypoints1,descriptors_1);
    descriptor->compute(img2,keypoints2,descriptors_2);
    //[3]对两幅图像中的BRIEF描述子进行匹配，使用Hamming距离
    vector<DMatch>match;
    matcher->match(descriptors_1,descriptors_2,match);
    //[4]匹配点筛选配对
    double min_dist=1000,max_dist=0;
    //找出所有匹配之间的最小距离和最大距离，等价于最相似和最不相似的两组点之间的距离
    for(int i=0;i<descriptors_1.rows;i++)
    {
        double dist=match[i].distance;
        if(dist<min_dist)min_dist=dist;
        if(dist>max_dist)max_dist=dist;
    }
    printf("--Max dist:%f\n",max_dist);
    printf("--Min dist:%f\n",min_dist);
    //当描述子之间的距离大于两倍的最小距离，认为匹配有误，但有时候最小距离会非常小，设置一个经验值30作为下限
    for(int i=0;i<descriptors_1.rows;i++)
    {
        if(match[i].distance<=max(2*min_dist,30.0))
        {
            matches.push_back(match[i]);
        }
    }
   // Mat img_match;
    //drawMatches ( img1, keypoints1, img2, keypoints2, matches, img_match );
   // imshow ( "所有匹配点对", img_match );
    //waitKey(0);



}
Point2d pixel2cam(const Point2d&p,const Mat&K) //像素坐标转相机归一化坐标
{
    return  Point2d(
            ( p.x-K.at<double>(0,2))/K.at<double>(0,0),
            (p.y-K.at<double>(1,2))/K.at<double>(1,1)
            );
}
void triangulation( const vector<KeyPoint>&keypoints_1, const vector<KeyPoint>&keypoints_2,
                    const std::vector<DMatch>&matches ,const Mat&R,const Mat&t,vector<Point3d>points)
{
    Mat T1=(Mat_<float>(3,4)<<
            1,0,0,0,
            0,1,0,0,
            0,0,1,0);
    Mat T2=(Mat_<float>(3,4)<<
            R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
            R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
            R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0));
    Mat K=(Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
    vector<Point2f>pts_1,pts_2;
    for(DMatch m:matches)
    {
        //像素坐标转相机坐标
        pts_1.push_back(keypoints_1[m.queryIdx].pt);
        pts_2.push_back(keypoints_2[m.trainIdx].pt);
    }
    Mat pts_4d;
    cv::triangulatePoints(T1,T2,pts_1,pts_2,pts_4d);


    //转换成非齐次坐标
    for(int i=0;i<pts_4d.cols;i++)
    {
        Mat x=pts_4d.col(i);
        x/=x.at<float>(3,0);//归一化
        Point3d p(
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
                );
        points.push_back(p);

    }
}