#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry> //提供旋转和平移的表示

/*Eigen几何模块的使用*/


int main(int argc,char**argv)
{
    //3D旋转矩阵直接使用Matrix3d或Matrix3f
    Eigen::Matrix3d rotation_matrix=Eigen::Matrix3d::Identity();
    //旋转向量使用AngleAxis,运算可以当做矩阵（因为重载了运算符）
    Eigen::AngleAxisd rotation_vector(M_PI/4,Eigen::Vector3d(0,0,1));//沿Z轴旋转45度
    cout.precision(3);


    cout<<"rotation matrix =\n"<<rotation_vector.matrix()<<endl;//用matrix()转换成矩阵
    rotation_matrix=rotation_vector.toRotationMatrix();//直接给旋转矩阵赋值

    //用旋转向量AngleAxis可以进行坐标变换
    Eigen::Vector3d v(1,0,0);
    Eigen::Vector3d v_rotated=rotation_vector*v;
    cout<<"(1,0,0)after rotation="<<v_rotated.transpose()<<endl;
    //用旋转矩阵MATRIX3d进行坐标变换
    v_rotated=rotation_matrix*v;
    cout<<"(1,0,0)after rotation ="<<v_rotated.transpose()<<endl;

    //欧拉角：可以将旋转矩阵直接转换成欧拉角
    Eigen::Vector3d euler_angles=rotation_matrix.eulerAngles(2,1,0);//zyx顺序
    cout<<"yaw pitch roll="<<euler_angles.transpose()<<endl;

    //欧式变换矩阵使用Eigen::Isometry
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();//虽然称为3d,实质上是4*4的矩阵
    T.rotate(rotation_vector);//按照rotation_vector进行旋转
    T.pretranslate(Eigen::Vector3d(1,3,4)); //把平移向量设置成（1.3.4）
    cout<<"Transform matrix= \n"<<T.matrix()<<endl;

    //用变换矩阵进行坐标变化
    Eigen::Vector3d v_transformed=T*v;
    cout<<" v transformed = "<<v_transformed.transpose()<<endl;

    //四元数
    //可以直接把旋转向量AngleAxis赋值给四元数，反之亦然
    Eigen::Quaterniond q =Eigen::Quaterniond(rotation_vector);
    cout<<"quaternioned =\n"<<q.coeffs()<<endl;//注意coeffs的顺序是（x,y,z,w)

    //把旋转矩阵赋值给四元数
    q=Eigen::Quaterniond(rotation_matrix);
    cout<<"quaternioned =\n"<<q.coeffs()<<endl;
    //使用四元数旋转一个向量，使用重载的乘法
    v_rotated=q*v;//注意数学上是qvq^{-1}
    cout<<"(1,0,0) after rotation="<<v_rotated.transpose()<<endl;

    return 0;
}