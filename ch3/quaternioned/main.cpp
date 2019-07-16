//已知相机的位姿用四元数表示为q=[0.35,0.2,0.3,0.1],顺序为x,y,z,w,编程实现 输出四元数对应的旋转矩阵，旋转矩阵的转置，
// 旋转矩阵的逆矩阵，旋转矩阵乘以自身的转置



#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc,char**argv)
{
   Quaterniond q=Quaterniond(0.35,0.2,0.3,0.1);//定义一个四元数并初始化顺序w,x,y,z
   q.normalize();//归一化 ，任意单位四元数才能表示旋转
   Matrix3d rotation_matrix;//定义旋转矩阵
   rotation_matrix=q.toRotationMatrix();//四元数转旋转矩阵

  cout<<"q coeffs is:"<<q.coeffs()<<endl;//四元数系数的输出顺序是x,y,z,w
   cout<<"rotation matrix:\n"<<rotation_matrix<<endl;
   cout<<"rotation matrix transpose:\n"<<rotation_matrix.transpose()<<endl;
   cout<<"rotation matrix inverse:\n"<<rotation_matrix.inverse()<<endl;
   cout<<"旋转矩阵乘以其转置：\n"<<rotation_matrix*rotation_matrix.transpose()<<endl;
    cout<<"旋转矩阵乘以其逆：\n"<<rotation_matrix*rotation_matrix.inverse()<<endl;


    return 0;
}