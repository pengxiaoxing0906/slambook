#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc,char*argv[])
{
    cout << "argc = " << argc << endl;
    if(argc == 3)
    {
        cout << argv[1] << "     " << argv[2] << endl;
    }
   if(argc!=3)
  {
       cout<<"啦啦啦啦 哈哈哈 wrong"<<endl;
       return 1;
  }

   //读取图像
   //Mat image1=imread("/home/pxx/Documents/slambook/ch7/feature_extraction/1.png",CV_LOAD_IMAGE_COLOR);
   //Mat image2=imread("/home/pxx/Documents/slambook/ch7/feature_extraction/2.png",CV_LOAD_IMAGE_COLOR);
   Mat image1=imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat image2=imread(argv[2],CV_LOAD_IMAGE_COLOR);


    //初始化
   std::vector<KeyPoint>keypoints_1,keypoints_2;
   Mat descriptors_1,descriptors_2;
   Ptr<ORB>orb=ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);

   //第一步：检测ORIENTED fast 角点位置
   orb->detect(image1,keypoints_1);
   orb->detect(image2,keypoints_2);

   //第二步：根据角点位置计算BRIEF算子
   orb->compute(image1,keypoints_1,descriptors_1);
   orb->compute(image2,keypoints_2,descriptors_2);

   Mat outimg1;
   drawKeypoints(image1,keypoints_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
   imshow("ORB特征点",outimg1);

   //第三步：对两幅图像中的BRIEF描述子进行匹配，使用Hamming距离
   vector<DMatch>matches;
   BFMatcher matcher(NORM_HAMMING);
   matcher.match(descriptors_1,descriptors_2,matches);

   //第四步：匹配点对筛选
   double min_dist=10000,max_dist=0;
   //找出所有匹配之间的最小距离和最大距离，即最相似的和最不相似的两组点之间的距离
   for(int i=0;i<descriptors_1.rows;i++)
   {
       double dist=matches[i].distance;
       if(dist<min_dist)min_dist=dist;
       if(dist>max_dist)max_dist=dist;
   }
   printf("--Max dist:%f\n",max_dist);
   printf("--Min dist:%f\n",min_dist);

   //当描述子之间的距离大于两倍的最小距离时，即认为匹配有误。
   //但有时候最小距离会非常小，设置一个经验值作为下限
   std::vector<DMatch>good_matches;
   for(int i=0;i<descriptors_1.rows;i++)
   {
       if(matches[i].distance<=max(2*min_dist,30.0))
       {
           good_matches.push_back(matches[i]);
       }
   }

   //第五步：绘制匹配结果
   Mat img_match;
   Mat img_goodmatch;
   drawMatches(image1,keypoints_1,image2,keypoints_2,matches,img_match);
   drawMatches(image1,keypoints_1,image2,keypoints_2,good_matches,img_goodmatch);
   imshow("所有匹配点对",img_match);
   imshow("优化后匹配点对",img_goodmatch);
   waitKey(0);
    return 0;
}