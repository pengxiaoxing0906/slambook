#include <iostream>
#include <ctime>
using namespace std;
//Eigen part
#include <Eigen/Core>
#include <Eigen/Dense>
#define MATRIX_SIZE 30

//

int main(int argc,char**argv)
{
    Eigen::Matrix<double,MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);
    Eigen::Matrix3d matrix_x=Eigen::Matrix3d::Random();

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        { cout<<matrix_NN(i,j)<<" ";}
        cout<<endl;
    };
    matrix_x = Eigen::Matrix3d::Identity();
    cout<<"赋值后的矩阵为:"<<endl;
    cout<<matrix_x<<endl;


    return 0;
}