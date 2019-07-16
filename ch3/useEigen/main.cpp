#include <iostream>
#include <ctime>
using namespace std;

//Eigen part
#include <Eigen/Core>
//稠密矩阵的代数运算（逆、特征值）
#include <Eigen/Dense>

#define MATRIX_SIZE 50





int main(int argc,char**argv)
{
   Eigen::Matrix<float,2,3>matrix_23; //declaration
   Eigen::Vector3d v_3d;
   Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>matrix_dynamic;
   Eigen::MatrixXd matrix_x;


   Eigen::Matrix3d matrix_33=Eigen::Matrix3d::Zero(); //initialize

   //input data
   matrix_23<<1,2,3,4,5,6;

   //output data
   cout<<matrix_23<<endl;
    v_3d<<3,2,1;

    //访问矩阵中的元素
   for(int i=0;i<1;i++)
   {
       for(int j=0;j<2;j++)
       {
           cout<<matrix_23(i,j)<<endl;
       }
   }

   //矩阵和向量相乘
   Eigen::Matrix<double,2,1>result=matrix_23.cast<double>()*v_3d;
   cout<<result<<endl;

   matrix_33=Eigen::Matrix3d::Random();
   cout<<matrix_33<<endl;

   cout<<matrix_33.transpose()<<endl;//转置
   cout<<matrix_33.sum()<<endl;//各元素的和
   cout<<matrix_33.trace()<<endl; //迹，主对角线上元素的和
   cout<<10*matrix_33<<endl;//数乘
   cout<<matrix_33.inverse()<<endl; //逆
   cout<<matrix_33.determinant()<<endl; //行列式

   //特征值
   //实对称矩阵可以保证对角化成功
   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>eigen_solver(matrix_33.transpose()*matrix_33);
   cout<<"Eigen value= "<<eigen_solver.eigenvectors()<<endl; //特征值
   /*特征值的个数运行出来是9个，但按照理论应该最多只有3个，3阶的呀*/
   cout<<"Eigen vectors= "<<eigen_solver.eigenvectors()<<endl;//特征向量


   //解方程 matrix_NN*x=v_Nd（两种方法）
   Eigen::Matrix<double,MATRIX_SIZE,MATRIX_SIZE>matrix_NN;
   matrix_NN=Eigen::MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);
   Eigen::Matrix<double,MATRIX_SIZE,1>v_Nd;
   v_Nd=Eigen::MatrixXd::Random(MATRIX_SIZE,1);

   clock_t time_stt=clock();//计时
   //直接求逆
   Eigen::Matrix<double,MATRIX_SIZE,1>x=matrix_NN.inverse()*v_Nd;
   cout<<"time use in normal inverse is "<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;

   //QR分解求逆 ，速度更快
   time_stt=clock();
   x=matrix_NN.colPivHouseholderQr().solve(v_Nd);
   cout<<"time use in QR composition is "<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;



    return 0;
}