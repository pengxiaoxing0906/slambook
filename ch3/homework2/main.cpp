#include <iostream>
#include <cmath>
using namespace std;

//Eigen part
#include <Eigen/Core>
//Eigen 几何模块
#include <Eigen/Geometry>
int main()
{
   //变量定义
    Eigen::Quaterniond Q1(0.35,0.2,0.3,0.1); //四元数表示（w,x,y,z)
    Eigen::Quaterniond Q2(-0.5,0.4,-0.1,0.2);
    Eigen::Vector3d T1(0.3,0.1,0.1);
    Eigen::Vector3d T2(-0.1,0.5,0.3);
    Eigen::Vector3d P1(0.5,0,0.2);  //在一号萝卜下的坐标
    Eigen::Vector3d Pw;//在世界坐标系下的坐标
    Eigen::Vector3d P2; //在二号小萝卜下的坐标


    //欧式变换矩阵使用Eigen::Isometry
    Eigen::Isometry3d T1W=Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T2W=Eigen::Isometry3d::Identity();


    //归一化
    Q1.normalize();
    Q2.normalize();


    //输出归一化参数
    cout<<"Q1 is "<<Q1.x()<<endl<<Q1.y()<<endl<<Q1.z()<<endl<<Q1.w()<<endl;
    cout<<"Q2 is "<<Q2.x()<<endl<<Q2.y()<<endl<<Q2.z()<<endl<<Q2.w()<<endl;

    cout<<"after normalize: "<<endl<<Q2.coeffs()<<endl;

    //设置变化矩阵的参数
    T1W.rotate(Q1);
    T1W.pretranslate(T1);
    T2W.rotate(Q2);
    T2W.pretranslate(T2);



    /*P1=T1W*Pw*/
    Pw=T1W.inverse()*P1;
    /*P2=T2W*Pw*/
    P2=T2W*Pw;

    //输出小萝卜二号下的该点坐标
    cout<<"改点在小萝卜二号下的坐标为："<<P2.transpose()<<endl;
    return 0;
}