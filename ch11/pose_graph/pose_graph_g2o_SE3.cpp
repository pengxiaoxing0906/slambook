#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

using namespace std;
/****
 * 本程序演示如何用g2o solver进行位姿图优化
 * sophere.g2o是人工生成的一个pose graph ,它是优化对象
 * 这里使用g2o/types/slam3d/中的SE3表示位姿，它实质上是四元数而非李代数
 * @return
 */

int main(int argc,char**argv)
{
    if(argc!=2)
    {
        cout<<"usage:pose_graph_g2o_SE3 sphere.g2o "<<endl;
        return 1;
    }
    ifstream fin(argv[1]);
    if(!fin)
    {
        cout<<"file "<<argv[1]<<" does not exit."<<endl;
        return 1;
    }

    //基本设置

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>>Block; //6*6块求解器 优化变量的维度R,t　观测值的维度R,t
    std::unique_ptr<Block::LinearSolverType>linearSolver(new g2o::LinearSolverCholmod<Block::PoseMatrixType>());//线性方程求解器
    std::unique_ptr<Block>solver_ptr(new Block(std::move(linearSolver)));
    //梯度下降方法，GN LM DogLeg
    g2o::OptimizationAlgorithmLevenberg*solver=new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    g2o::SparseOptimizer optimizer; //图模型
    optimizer.setAlgorithm(solver); //设置求解器

    //vertex and edges

    int vertexCnt=0,edgeCnt=0;
    while(!fin.eof())
    {
        string name;
        fin>>name;
        if(name=="VERTEX_SE3:QUAT")
        {
            //SE3顶点
            g2o::VertexSE3* v=new g2o::VertexSE3();
            int index=0;
            fin>>index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if(index==0)
                v->setFixed(true);
        }
        else if(name=="EDGE_SE3:QUAT")
        {
            //SE3-SE3 edge
            g2o::EdgeSE3* e=new g2o::EdgeSE3();
            int idx1,idx2;
            fin>>idx1>>idx2;
            e->setId(edgeCnt++);
            e->setVertex(0,optimizer.vertices()[idx1]);
            e->setVertex(1,optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
        }
        if(!fin.good())break;
    }

    cout<<"read total "<<vertexCnt<<" vertices, "<<edgeCnt<<" edges."<<endl;

    cout<<"prepare optimizing..."<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    cout<<"calling optimizing ..."<<endl;
    optimizer.optimize(30);

    cout<<"saving optimization results ..."<<endl;
    optimizer.save("result.g2o");
    return 0;
}