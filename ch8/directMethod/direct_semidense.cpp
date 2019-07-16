#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>  //头文件中包含一些定义的符号常量

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace g2o;
using namespace cv;


/*RGBD的稀疏直接法*/
//一次测量的值，包括一个世界坐标系下的三维点和一个灰度值
struct Measurement
{
    Eigen::Vector3d pos_world;
    float grayscale;
    Measurement(Eigen::Vector3d p, float g):pos_world(p),grayscale(g){}

};
inline Eigen::Vector3d project2Dto3D(int x,int y,int d, float fx, float fy,float cx,float cy,float scale)
{
    float zz=float(d)/scale;
    float xx=zz*(x-cx)/fx;
    float yy=zz*(y-cy)/fy;
    return Eigen::Vector3d(xx,yy,zz);
}
inline Eigen::Vector2d project3Dto2D(float x, float y, float z, float fx, float fy, float cx, float cy)
{
    float u=x*fx/z+cx;
    float v=y*fy/z+cy;
    return Eigen::Vector2d(u,v);
}

//直接法估计位姿
//输入：测量值（空间点的灰度）、新的灰度图、相机内参；输出：相机位姿
bool poseEstimationDirect(const vector<Measurement>&measurements,cv::Mat *gray,Eigen::Matrix3f&K,Eigen::Isometry3d&Tcw);

//直接法的误差是光度误差 间接法的是特征点的重投影误差
class EdgeSE3ProjectDirect:public BaseUnaryEdge<1,double,VertexSE3Expmap>
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d x_world_;//3D point in world frame
    float cx_=0,cy_=0;
    float fx_=0,fy_=0;//Camera intrinsics
    cv::Mat*image_= nullptr;
    EdgeSE3ProjectDirect(){}
    EdgeSE3ProjectDirect(Eigen::Vector3d point,float fx,float fy,float cx,float cy,cv::Mat*image)
            :x_world_(point),fx_(fx),fy_(fy),cx_(cx),cy_(cy),image_(image){}
    virtual void computeError()
    {
        const VertexSE3Expmap*v= static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d x_local=v->estimate().map(x_world_);
        float x=x_local[0]*fx_/x_local[2]+cx_; //求像素坐标u
        float y=x_local[1]*fy_/x_local[2]+cy_; //求像素坐标v
        //check x,y is in the image
        if(x-4<0||(x+4)>image_->cols||(y-4)<0||(y+4)>image_->rows)
        {
            _error(0,0)=0.0;
            this->setLevel(1);
        }
        else
        {
            _error(0,0)=getPixelValue(x,y)-_measurement;
        }
    }
    virtual void linearizeOplus()
    {
        if(level()==1)
        {
            _jacobianOplusXi=Eigen::Matrix<double,1,6>::Zero();
            return;
        }
        VertexSE3Expmap*vtx=static_cast<VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d xyz_trans=vtx->estimate().map(x_world_);
        double x=xyz_trans[0];
        double y=xyz_trans[1];
        double invz=1.0/xyz_trans[2];
        double invz_2=invz*invz;

        float u=x*fx_*invz+cx_;
        float v=y*fy_*invz+cy_;

        Eigen::Matrix<double ,2,6 >jacobian_uv_ksai;
        jacobian_uv_ksai(0,0)=-x*y*invz_2*fx_;
        jacobian_uv_ksai(0,1)=(1+(x*x*invz_2))*fx_;
        jacobian_uv_ksai(0,2)=-y*invz*fx_;
        jacobian_uv_ksai(0,3)=invz*fx_;
        jacobian_uv_ksai(0,4)=0;
        jacobian_uv_ksai(0,5)=-x*invz_2*fx_;

        jacobian_uv_ksai(1,0)=-(1+y*y*invz_2)*fy_;
        jacobian_uv_ksai(1,1)=x*y*invz_2*fy_;
        jacobian_uv_ksai(1,2)=x*invz*fy_;
        jacobian_uv_ksai(1,3)=0;
        jacobian_uv_ksai(1,4)=invz*fy_;
        jacobian_uv_ksai(1,5)=-y*invz_2*fy_;

        Eigen::Matrix<double,1,2>jacobian_pixel_uv;
        jacobian_pixel_uv(0,0)=(getPixelValue(u+1,v)-getPixelValue(u-1,v))/2;
        jacobian_pixel_uv(0,1)=(getPixelValue(u,v+1)-getPixelValue(u,v-1))/2;

        _jacobianOplusXi=jacobian_pixel_uv*jacobian_uv_ksai;

    }
    virtual bool read(std::istream& in){}
    virtual bool write(std::ostream& out)const{}

protected:
    //get a gray scale value from reference image(bilinear interpolated)双线性插值
    inline float getPixelValue(float x,float y)
    {
        uchar *data=&image_->data[int(y)*image_->step+int(x)];
        float xx=x-floor(x); //floor()函数功能是向下取整，直接取按照数轴上最接近要求值的左边值
        float yy=y-floor(y);
        return float(
                (1-xx)*(1-yy)*data[0]+xx*(1-yy)*data[1]+(1-xx)*yy*data[image_->step]+xx*yy*data[image_->step+1]
        );
    }


};


int main(int argc,char**argv) {
    if(argc!=2)
    {
        cout<<"usage:useLK path_to_dataset"<<endl;
        return 1;
    }
    srand((unsigned int)time(0));
    string path_to_dataset=argv[1];
    string associate_file=path_to_dataset+"/associate.txt";
    ifstream fin(associate_file);

    string rgb_file,depth_file,time_rgb,time_depth;
    cv::Mat color,depth,gray;
    vector<Measurement>measurements;
    //相机内参
    float cx=325.5;
    float cy=253.5;
    float fx=518.0;
    float fy=519.0;
    float depth_scale=1000.0;
    Eigen::Matrix3f K;
    K<<fx,0.f,cx,0.f,fy,cy,0.f,0.f,1.0f;

    Eigen::Isometry3d Tcw=Eigen::Isometry3d::Identity();
    cv::Mat prev_color;
    //以第一个图像为参考，对后续图像和参考图像做直接法 每次都是后面的图跟第一张图做比较
    for(int index=0;index<10;index++)
    {
        cout<<"*********loop"<<index<<"********"<<endl;
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
        color=cv::imread(path_to_dataset+"/"+rgb_file);
        depth=cv::imread(path_to_dataset+"/"+depth_file,-1);
        if(color.data==nullptr||depth.data== nullptr)
            continue;
        cv::cvtColor(color,gray,cv::COLOR_BGR2GRAY);
        if(index==0)
        {
            //对第一帧提取FAST特征点
            vector<KeyPoint>keypoints;
            cv::Ptr<cv::FastFeatureDetector>detector=cv::FastFeatureDetector::create();
            detector->detect(color,keypoints);
            for(int x=10;x<gray.cols-10;x++)
            {
                for(int y=10;y<gray.rows-10;y++)
                {Eigen::Vector2d delta(gray.ptr<uchar>(y)[x+1]-gray.ptr<uchar>(y)[x+1],
                        gray.ptr<uchar>(y+1)[x]-gray.ptr<uchar>(y-1)[x]);
                if(delta.norm()<50)
                    continue;
                    ushort d = depth.ptr<ushort>(y)[x];//cvRound 返回跟参数最接近的整数值
                    if (d == 0)
                        continue;
                    Eigen::Vector3d p3d = project2Dto3D(x, y, d, fx, fy, cx, cy, depth_scale);
                    float grayscale = float(gray.ptr<uchar>(y)[x]);
                    measurements.push_back(Measurement(p3d, grayscale));
                }
            }
            prev_color=color.clone();
            continue;
        }
        //使用直接法计算相机运动
        chrono::steady_clock::time_point t1=chrono::steady_clock::now();
        poseEstimationDirect ( measurements, &gray, K, Tcw );

        chrono::steady_clock::time_point t2=chrono::steady_clock::now();
        chrono::duration<double>time_used=chrono::duration_cast<chrono::duration<double>>(t2-t1);
        cout<<"direct method costs time:"<<time_used.count()<<" seconds."<<endl;
        cout<<"Tcw="<<Tcw.matrix()<<endl;

        //plot the feature points
        cv::Mat img_show(color.rows*2,color.cols,CV_8UC3);
        prev_color.copyTo(img_show(cv::Rect(0,0,color.cols,color.rows)));
        color.copyTo(img_show(cv::Rect(0,color.rows,color.cols,color.rows)));
        for(Measurement m:measurements)
        {
            if(rand()>RAND_MAX/5)
                continue;
            Eigen::Vector3d p=m.pos_world;
            Eigen::Vector2d pixel_prev=project3Dto2D(p(0,0),p(1,0),p(2,0),fx,fy,cx,cy);
            //Eigen::Vector3d p2 = Tcw*m.pos_world;//不理解原来代码这一句 维度不对应怎么相乘
            Eigen::Vector3d p2=Tcw.rotation()*m.pos_world+Tcw.translation(); //两种写法都可以实现
            Eigen::Vector2d pixel_now=project3Dto2D(p2(0,0),p2(1,0),p2(2,0),fx,fy,cx,cy);
            if(pixel_now(0,0)<0||pixel_now(0,0)>=color.cols||pixel_now(1,0)<0||pixel_now(1,0)>=color.rows)
                continue;
            float b=255*float(rand())/RAND_MAX;
            float g=255*float(rand())/RAND_MAX;
            float r=255*float(rand())/RAND_MAX;
            cv::circle(img_show,cv::Point2d(pixel_prev(0,0),pixel_prev(1,0)),8,cv::Scalar(b,g,r),2); //先cols再rows
            cv::circle(img_show,cv::Point2d(pixel_now(0,0),pixel_now(1,0)+color.rows),8,cv::Scalar(b,g,r),2);
            cv::line(img_show,cv::Point2d(pixel_prev(0,0),pixel_prev(1,0)),cv::Point2d(pixel_now(0,0),pixel_now(1,0)+color.rows),cv::Scalar(b,g,r),1);
        }
        cv::imshow("result",img_show);
        cv::waitKey(0);

    }

    return 0;
}
bool poseEstimationDirect(const vector<Measurement>&measurements,Mat *gray,Eigen::Matrix3f&K,Eigen::Isometry3d&Tcw)
{
    //初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>DirectBlock;//求解向量是6*1维的
    std::unique_ptr<DirectBlock::LinearSolverType>linearSolver(new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>());
    std::unique_ptr<DirectBlock>solver_ptr(new DirectBlock(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg*solver=new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    //vertex
    g2o::VertexSE3Expmap*pose=new g2o::VertexSE3Expmap();
    pose->setEstimate(g2o::SE3Quat(Tcw.rotation(),Tcw.translation()));
    pose->setId(0);
    optimizer.addVertex(pose);

    //edge
    int id=1;
    for(Measurement m:measurements)
    {
        EdgeSE3ProjectDirect*edge=new EdgeSE3ProjectDirect(m.pos_world,K(0,0),K(1,1),K(0,2),K(1,2),gray);
        edge->setVertex(0,pose);
        edge->setMeasurement(m.grayscale);
        edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
        edge->setId(id++);
        optimizer.addEdge(edge);

    }
    cout<<"edge in graph: "<<optimizer.edges().size()<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    Tcw=pose->estimate();
}

