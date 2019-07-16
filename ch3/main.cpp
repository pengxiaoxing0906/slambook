#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;


int main() {
    // 旋转向量（轴角）：沿Z轴旋转45°
    Eigen::AngleAxisd rotation_vector = {M_PI/4, Eigen::Vector3d(0, 0, 1)};
    cout<<"rotation_vector axis = " << rotation_vector.axis().transpose() << "\nrotation_vector angle = " << rotation_vector.angle()<<endl;

    //旋转矩阵：沿Z轴旋转45°
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << cos(M_PI/4), -sin(M_PI/4), 0,
                       sin(M_PI/4), cos(M_PI/4), 0,
                       0, 0, 1;
    cout<<"rotation matrix =\n" << rotation_matrix <<endl;

    // 四元数：沿Z轴旋转45°
    Eigen::Quaterniond quat = {cos(M_PI/8), 0, 0, sin(M_PI/8)};
    cout<<"四元数输出方法1：quaternion =\n" << quat.coeffs().transpose() << endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    cout<<"四元数输出方法2：\n x = " << quat.x() << "\n y = " << quat.y() << "\n z = " << quat.z() << "\n w = " << quat.w() << endl;

    // 欧拉角: ：沿Z轴旋转45°
    Eigen::Vector3d euler_angles = Eigen::Vector3d(M_PI/4, 0, 0);// ZYX顺序，即roll pitch yaw顺序
    cout << "Euler: yaw pitch roll = " << euler_angles.transpose() << endl;


    //  相互转化关系
    // ----------  请在下面补充对应的代码 ----------
    // 旋转向量转化为其他形式
    cout<<"旋转向量转化为旋转矩阵方法1：rotation matrix =\n" << rotation_vector.toRotationMatrix() << endl;
    cout<<"旋转向量转化为旋转矩阵方法2：rotation matrix =\n" << rotation_vector.matrix() <<endl;

    quat = Eigen::Quaterniond(rotation_vector);
    cout << "旋转向量转化为四元数：quaternion =\n" << quat.coeffs().transpose() <<endl;  //coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部


    // 旋转矩阵转化为其他形式
    rotation_vector = rotation_vector.fromRotationMatrix(rotation_matrix);
    cout<<"旋转矩阵转化为旋转向量：rotation_vector axis = \n" << rotation_vector.axis().transpose() <<"\n rotation_vector angle = "<< rotation_vector.angle() << endl;
    //注意：fromRotationMatrix 参数只适用于旋转向量，不适用于四元数

    rotation_vector =  Eigen::AngleAxisd(rotation_matrix);
    cout<<"旋转矩阵直接给旋转向量赋值初始化：rotation_vector axis = \n" << rotation_vector.axis().transpose() << "\n rotation_vector angle = "<< rotation_vector.angle() << endl;

    euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout<<"旋转矩阵转化为欧拉角：yaw pitch roll = "<< euler_angles.transpose() << endl;

    quat = Eigen::Quaterniond(rotation_matrix);
    cout<<"旋转矩阵转化为四元数：quaternion =\n"<< quat.coeffs().transpose() << endl;

    // 四元数转化为其他形式
    rotation_vector = Eigen::AngleAxisd(quat);
    cout << "四元数转化为旋转向量：rotation_vector axis = \n" << rotation_vector.axis().transpose() <<"\n rotation_vector angle = "<< rotation_vector.angle() << endl;

    rotation_matrix = quat.toRotationMatrix();
    cout<<"四元数转化为旋转矩阵方法1：rotation matrix =\n"<< rotation_matrix << endl;

    rotation_matrix = Eigen::Matrix3d(quat);
    cout<<"四元数转化为旋转矩阵方法2：rotation matrix =\n"<< rotation_matrix << endl;


    //  欧氏变换矩阵
    // 欧氏变换矩阵使用 Eigen::Isometry，仿射变换用 Eigen::Affine3d，射影变换用 Eigen::Projective3d
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();              // 名称为3d，实质上是4 x 4的矩阵
    T.rotate(quat);
    T.pretranslate(Eigen::Vector3d(1, 2, 3));
    cout << "Transform matrix = \n" << T.matrix() << endl;             // 变换矩阵需要用成员函数转一下输出

    cout << "欧氏变化矩阵提取旋转矩阵：rotation_matrix = \n" << T.rotation() << endl;
    cout << "欧氏变化矩阵提取平移向量：translation = \n" << T.translation() << endl;
    return 0;
}