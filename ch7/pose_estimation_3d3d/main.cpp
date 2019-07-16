#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

using namespace std;
using namespace cv;


void find_feature_match(const Mat&img1,const Mat&img2,
        vector<KeyPoint>&keypoints1,
        vector<KeyPoint>&keypoints2,
        vector<DMatch>&matches);
Point2d pixel_cam(const Point2d&p,const  Mat&k);
void estimation_3d3d(vector<Point3f>&pts1,
        vector<Point3f>&pts2,
        Mat&R,Mat&t);
void bundleAdjustment(const vector<Point3f>points1_3d,
        const vector<Point3f>points2_3d,
        Mat&R,Mat&t);

//g2o edge
class EdgeProjectXYZRGBDPoseOnly:public g2o::BaseUnaryEdge<3,Eigen::Vector3d,g2o::VertexSE3Expmap>
{
protected:
    Eigen::Vector3d _point;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d&point):_point(point){}
    virtual void computeError()
    {
        const g2o::VertexSE3Expmap*pose= static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        //measurement is p,point is p'
        _error=_measurement-pose->estimate().map(_point);
    }
    virtual  void linearizaeOplus()
    {
        g2o::VertexSE3Expmap*pose= static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans=T.map(_point);
        double x=xyz_trans[0];
        double y=xyz_trans[1];
        double z=xyz_trans[2];

        _jacobianOplusXi(0,0)=0;
        _jacobianOplusXi(0,1)=-z;
        _jacobianOplusXi(0,2)=y;
        _jacobianOplusXi(0,3)=-1;
        _jacobianOplusXi(0,4)=0;
        _jacobianOplusXi(0,5)=0;

        _jacobianOplusXi(1,0)=z;
        _jacobianOplusXi(1,1)=0;
        _jacobianOplusXi(1,2)=-x;
        _jacobianOplusXi(1,3)=0;
        _jacobianOplusXi(1,4)=-1;
        _jacobianOplusXi(1,5)=0;

        _jacobianOplusXi(2,0)=-y;
        _jacobianOplusXi(2,1)=x;
        _jacobianOplusXi(2,2)=0;
        _jacobianOplusXi(2,3)=0;
        _jacobianOplusXi(2,4)=0;
        _jacobianOplusXi(2,5)=-1;

    }
    bool read (istream& in){}
    bool write(ostream& out)const{}
};

int main(int argc,char**argv)
{

    //[1]检查参数argc
    if(argc!=5)
    {
        cout<<"传入数据有错误哈哈"<<endl;
        return 1;
    }
    //[2]读取彩色图
    Mat img1=imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img2=imread(argv[2],CV_LOAD_IMAGE_COLOR);
    //[3]定义传入特征点匹配的参数
    vector<KeyPoint>keypoints1,keypoints2;
    vector<DMatch>matches;
    //[4]调用特征点匹配函数并显示出匹配点对的数量
    find_feature_match(img1,img2,keypoints1,keypoints2,matches);
    cout<<"匹配点对数量"<<matches.size()<<endl;
    //[5]读取深度图
    Mat depth1=imread(argv[3],CV_LOAD_IMAGE_UNCHANGED);//深度图为16位无符号数，单通道图像
    Mat depth2=imread(argv[4],CV_LOAD_IMAGE_UNCHANGED);
    //[6]定义内参K和三维点数组
    Mat K=(Mat_<double>(3,3)<<520.9,0,325.1, 521.0,0,249.7, 0,0,1);
    vector<Point3f>pts1,pts2;
    //[7]结合彩色图和深度图信息形成三维点并输出三维点的size
    for(DMatch m:matches)
    {
        ushort d1=depth1.ptr<unsigned short>(int(keypoints1[m.queryIdx].pt.y))[int(keypoints1[m.queryIdx].pt.x)];
        ushort d2=depth2.ptr<unsigned short>(int(keypoints2[m.trainIdx].pt.y))[int(keypoints2[m.trainIdx].pt.x)];
    /*    if(!depth1.data)
        {
            cout<<"深度图1 读取失败"<<endl;
            return 1;
        }
        if(!depth2.data)
        {
            cout<<"深度图2 读取失败"<<endl;
            return 1;
        }
        */
        if(d1==0||d2==0)
            continue;
        Point2d p1=pixel_cam(keypoints1[m.queryIdx].pt,K);
        Point2d p2=pixel_cam(keypoints2[m.trainIdx].pt,K);
      //  cout<<"p1 value"<<p1<<endl;
      //  cout<<"p2 value"<<p1<<endl;
        float dd1=float(d1)/5000.0;
        float dd2=float(d2)/5000.0;
        pts1.push_back(Point3f(p1.x*dd1,p1.y*dd1,dd1));
        pts2.push_back(Point3f(p2.x*dd2,p2.y*dd2,dd2));
       // cout<<"pts1 value "<<pts1<<endl;
       // cout<<"pts2 value "<<pts2<<endl;

    }
    cout<<"3d3d pairs:"<<pts1.size()<<endl;
    Mat R, t;
    //[8]利用获取的三维点调用pose_estimation_3d3d计算出R和T，并输出R和T的信息
    estimation_3d3d(pts1,pts2,R,t);
    cout<<" ICP via SVD results:"<<endl;
    cout<<"R="<<R<<endl;
    cout<<"t="<<t<<endl;
    cout<<"R_inv="<<R.t()<<endl;
    cout<<"t_inv="<<-R.t()*t<<endl;
    //[9]调用BA优化优化所求变量R和T
    cout<<"calling bundle adjustment"<<endl;
    bundleAdjustment(pts1,pts2,R,t);
    //[10]验证pts1=R*pts2+t
    for(int i=0;i<5;i++)
    {
        cout<<"p1= "<<pts1[i]<<endl;
        cout<<"p2= "<<pts2[i]<<endl;
        cout<<"(R*p2+t)= "<<R*(Mat_<double>(3,1)<<pts2[i].x,pts2[i].y,pts2[i].z)+ t<<endl;
        cout<<endl;
    }

    return 0;
}
void find_feature_match(const Mat&img1,const Mat&img2,
                        vector<KeyPoint>&keypoints1,
                        vector<KeyPoint>&keypoints2,
                        vector<DMatch>&matches)
{
    //[1]定义描述子，初始化
    Mat descriptors1,descriptors2;
    Ptr<FeatureDetector>detector=ORB::create();
    Ptr<DescriptorExtractor>descriptor=ORB::create();
    Ptr<DescriptorMatcher>matcher=DescriptorMatcher::create("BruteForce-Hamming");
    //[2]检测角点位置
    detector->detect(img1,keypoints1);
    detector->detect(img2,keypoints2);
    //[3]计算描述子
    descriptor->compute(img1,keypoints1,descriptors1);
    descriptor->compute(img2,keypoints2,descriptors2);
    //[4]进行匹配
    vector<DMatch>match;
    matcher->match(descriptors1,descriptors2,match);
    //[5]匹配点筛选配对(先找出最大和最小的描述子之间的距离，再利用判断条件进行筛选）
    double min_dist=1000,max_dist=0;
    for(int i=0;i<descriptors1.rows;i++)
    {
        double dist=match[i].distance;
        if(dist>max_dist)max_dist=dist;
        if(dist<min_dist)min_dist=dist;
    }
    printf("max dist:%f\n ",max_dist);
    printf("min dist: %f\n",min_dist);
    for(int i=0;i<descriptors1.rows;i++)
    {
        if(match[i].distance<max(2*min_dist,30.0))
            matches.push_back(match[i]);
    }
}
Point2d pixel_cam(const Point2d&p,const  Mat&k)
{
    //cout<<"K="<<k<<endl;
    //cout<<p.x<<" "<<p.y<<endl;
   // Mat K=(Mat_<double>(3,3)<<520.9,0,325.1, 521.0,0,249.7, 0,0,1);
    double cx=325.1;
    double cy=249.7;
    double fx=520.9;
    double fy=521.0;

   // cout<<"x value "<<(p.x-cx)/fx<<" y value "<<(p.y-cy)/fy<<endl;

    return Point2d (
            (p.x-cx)/fx,
            (p.y-cy)/fy
                    );
}
void estimation_3d3d(vector<Point3f>&pts1,
                     vector<Point3f>&pts2,
                     Mat&R,Mat&t)
{
    /*使用SVD分解求出R和t */
    Point3f p1,p2;
    int N=pts1.size();
    for(int i=0;i<N;i++)
    {
        p1+=pts1[i];
        p2+=pts2[i];
    }
    p1=Point3f(Vec3f(p1)/N);
    p2=Point3f(Vec3f(p2)/N);
    vector<Point3f>q1(N),q2(N);//remove the center
    for(int i=0;i<N;i++)
    {
      q1[i]=pts1[i]-p1;
      q2[i]=pts2[i]-p2;
    }
    //compute q1*q2^
    Eigen::Matrix3d W=Eigen::Matrix3d::Zero();
    for(int i=0;i<N;i++)
    {
        W+=Eigen::Vector3d(q1[i].x,q1[i].y,q1[i].z)*Eigen::Vector3d(q2[i].x,q2[i].y,q2[i].z).transpose();
    }
    cout<<"w:"<<W<<endl;
    //SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d>svd(W,Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U=svd.matrixU();
    Eigen::Matrix3d V=svd.matrixV();
    if(U.determinant()*V.determinant()<0)
    {
        for(int x=0;x<3;++x)
        {
            U(x,2)*=-1;
        }
    }
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    Eigen::Matrix3d R_=U*(V.transpose());
    Eigen::Vector3d t_=Eigen::Vector3d(p1.x,p1.y,p1.z)-R_*Eigen::Vector3d(p2.x,p2.y,p2.z);

    //convert to cv::Mat
    R=(Mat_<double>(3,3)<<R_(0,0),R_(0,1),R_(0,2),
            R_(1,0),R_(1,1),R_(1,2),
            R_(2,0),R_(2,1),R_(2,2));
    t=(Mat_<double>(3,1)<<t_(0,0),t_(1,0),t_(2,0));

}
void bundleAdjustment(const vector<Point3f>points1_3d,
                      const vector<Point3f>points2_3d,
                      Mat&R,Mat&t)
{
    //[1]初始化
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>>Block; //pose的维度为6，landmark的维度为3;
    std::unique_ptr<Block::LinearSolverType>linearSolver(new g2o::LinearSolverEigen<Block::PoseMatrixType>());//线性方程求解器
    std::unique_ptr<Block>solver_ptr(new Block (std::move(linearSolver)));//矩阵块求解器
    g2o::OptimizationAlgorithmGaussNewton*solver=new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    //[2]add vertex;
    g2o::VertexSE3Expmap*pose=new g2o::VertexSE3Expmap();//camera pose
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0)));
    optimizer.addVertex(pose);

    //[3]add edges
    int index=1;
    vector<EdgeProjectXYZRGBDPoseOnly*>edges;
    for(size_t i=0;i<points1_3d.size();i++)
    {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(
                Eigen::Vector3d(points2_3d[i].x, points2_3d[i].y, points2_3d[i].z) );
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(pose));
        edge->setMeasurement(Eigen::Vector3d(points1_3d[i].x,points1_3d[i].y,points1_3d[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity()*1e4);
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);

    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;
}