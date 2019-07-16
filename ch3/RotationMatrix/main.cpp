/*已知旋转矩阵定义是沿着Z轴旋转45°。请按照该定义初始化旋转向量、旋转矩阵、四元数、欧拉角。请编程实现：
1、以上四种表达方式的相互转换关系并输出，并参考给出的结果验证是否正确。
2、假设平移向量为（1,2,3）,请输出旋转矩阵和该平移矩阵构成的欧式变换矩阵，并根据欧式变换矩阵提取旋转向量及平移向量。*/



#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

int main(int argc,char**argv)
{
    //initialize
    //旋转向量，沿Z轴旋转45度
    Eigen::AngleAxisd rotation_vector(M_PI/4,Eigen::Vector3d(0,0,1));
    cout<<"rotation_vector axis =\n"<<rotation_vector.axis()<<"\n rotation_vector angle = "<<rotation_vector.angle()<<endl;

    //旋转矩阵：沿Z轴旋转45度
    Eigen::Matrix3d rotation_matrix=Eigen::Matrix3d::Identity();
    rotation_matrix << cos(M_PI/4), -sin(M_PI/4), 0,
            sin(M_PI/4), cos(M_PI/4), 0,
            0, 0, 1;
    cout<<"rotation_matrix =\n"<<rotation_matrix<<endl;

    //四元数
    Eigen::Quaterniond q=Eigen::Quaterniond(0,0,0.383,0.924);
    cout<<"q 的系数是："<<q.coeffs()<<endl;//coeffs的顺序是（x,y,z,w),w为实部，前三者为虚部

    //欧拉角
    Eigen::Vector3d euler_angles=Eigen::Vector3d(M_PI/4,0,0);//ZYX顺序
    cout<<"euler:yaw pitch roll="<<euler_angles.transpose()<<endl;

    //相互转换关系


    //旋转向量转换为其他形式
    cout<<"旋转向量转化为旋转矩阵方法1：rotation matrix=\n"<<rotation_vector.toRotationMatrix()<<endl;
    cout<<"旋转向量转化为旋转矩阵方法2：rotation matrix=\n"<<rotation_vector.matrix()<<endl;
   // q=rotation_vector;
    q = Eigen::Quaterniond(rotation_vector);
    cout<<"旋转向量转四元数：quaternion q =\n"<<q.coeffs().transpose()<<endl;



    //旋转矩阵转化为其他形式
    rotation_vector=rotation_vector.fromRotationMatrix(rotation_matrix);
    cout<<"旋转矩阵转换成旋转向量 rotation_vector axis=\n"<<rotation_vector.axis().transpose()<<"\n rotation vector angle" <<rotation_vector.angle()<<endl;
    //rotation_vector=rotation_matrix;
    rotation_vector =  Eigen::AngleAxisd(rotation_matrix);
    cout<<"旋转矩阵直接给旋转向量赋值初始化：rotation_vector axis=\n"<<rotation_vector.axis().transpose()<<"rotation_vector angle="<<rotation_vector.angle()<<endl;
    //euler_angles=rotation_matrix;
    euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout<<"旋转矩阵转换成欧拉角：yaw pitch roll="<<euler_angles.transpose()<<endl;
   // q=rotation_matrix;
    q = Eigen::Quaterniond(rotation_matrix);
    cout<<"旋转矩阵转化成四元数：quaternione=\n"<<q.coeffs()<<endl;

    //四元数转换成其他形式
   // rotation_vector=q;
    rotation_vector = Eigen::AngleAxisd(q);
    cout<<"四元数转换成为旋转向量 rotation_vector axis=\n"<<rotation_vector.axis().transpose()<<", rotation_vector angle= "<<rotation_vector.angle()<<endl;
    rotation_matrix=q.toRotationMatrix();
    cout<<"四元数转换成旋转矩阵方法1： rotation matrix =\n"<<rotation_matrix.transpose()<<endl;
    rotation_matrix=q.matrix();
    //rotation_matrix = Eigen::Matrix3d(q);
    cout<<"四元数转换成旋转矩阵方法2： rotation matrix =\n"<<rotation_matrix.transpose()<<endl;

    //欧式变换矩阵
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    cout<<"欧式变化矩阵提取旋转矩阵 rotation_matrix=\n";
    T.rotate(q);
    cout<<"欧式变换矩阵提取平移向量 translation=\n";
    T.pretranslate(Eigen::Vector3d(1,3,4));
    cout<<"transform matrix=\n"<<T.matrix()<<endl;
    cout << "欧氏变化矩阵提取旋转矩阵：rotation_matrix = \n" << T.rotation() << endl;
    cout << "欧氏变化矩阵提取平移向量：translation = \n" << T.translation() << endl;



    return 0;
}