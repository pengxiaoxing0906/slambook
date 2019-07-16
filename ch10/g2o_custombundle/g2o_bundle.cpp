#include <Eigen/StdVector>
#include <Eigen/Core>

#include <iostream>
#include <stdint.h>

#include <unordered_set>
#include <memory>
#include <vector>
#include <stdlib.h>

#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "g2o/solvers/structure_only/structure_only_solver.h"

#include "common/BundleParams.h"
#include "common/BALProblem.h"
#include "g2o_bal_class.h"

using namespace std;
using namespace Eigen;
typedef Eigen::Map<Eigen::VectorXd>VectorRef;
typedef Eigen::Map<const Eigen::VectorXd>ConstVectorRef;
typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3>>BalBlockSolver;

//set up the vertexs and edges for the bundle adjustment
void BuildProblem(const BALProblem*bal_problem,g2o::SparseOptimizer*optimizer,const BundleParams&params)
{
    const int num_points = bal_problem->num_points();
    const int num_cameras = bal_problem->num_cameras();
    const int camera_block_size = bal_problem->camera_block_size();
    const int point_block_size = bal_problem->point_block_size();


//set camera vertxe with initial value in the dataset
    const double *raw_cameras = bal_problem->cameras();
    for(int i=0;i<num_cameras;i++)
    {
        ConstVectorRef temVecCamera(raw_cameras+camera_block_size*i,camera_block_size); //这一句是干嘛的　括号里这样写表示什么意思
        VertexCameraBAL*pCamera=new VertexCameraBAL();
        pCamera->setEstimate(temVecCamera);//设置初值
        pCamera->setId(i);

        optimizer->addVertex(pCamera);


    }

    //set point vertex with initial value in the dataset
    const  double*raw_points=bal_problem->points();
    for(int j=0;j<num_points;j++)
    {
        ConstVectorRef temVecPoint(raw_points+point_block_size*j,point_block_size);//同上　为啥要这样写
        VertexPointBAL*pPoint=new VertexPointBAL();
        pPoint->setEstimate(temVecPoint);
        pPoint->setId(j+num_cameras);

        pPoint->setMarginalized(true); //shur elimination
        optimizer->addVertex(pPoint);
    }

    //set edge for graph
    const int num_observations=bal_problem->num_observations();
    const double*observations=bal_problem->observations();
    for(int i=0;i<num_observations;i++)
    {
        EdgeObservationBAL*bal_edge=new EdgeObservationBAL();

        const int camera_id=bal_problem->camera_index()[i];//get id for the camera
        const int point_id=bal_problem->point_index()[i]+num_cameras;//get id for the point

        if(params.robustify)
        {
            g2o::RobustKernelHuber*rk=new g2o::RobustKernelHuber;
            rk->setDelta(1.0);
            bal_edge->setRobustKernel(rk);
        }

        bal_edge->setVertex(0,dynamic_cast<VertexCameraBAL*>(optimizer->vertex(camera_id)));
        bal_edge->setVertex(1, dynamic_cast<VertexPointBAL*>(optimizer->vertex(point_id)));
        bal_edge->setInformation(Eigen::Matrix2d::Identity()); //存储协方差矩阵的逆
        bal_edge->setMeasurement(Eigen::Vector2d(observations[2*i+0],observations[2*i+1])); //方括号里的表达式是啥意思


        optimizer->addEdge(bal_edge);
    }
}

void WriteToBALProblem(BALProblem* bal_problem,g2o::SparseOptimizer*optimizer)
{
    const int num_points = bal_problem->num_points();
    const int num_cameras = bal_problem->num_cameras();
    const int camera_block_size = bal_problem->camera_block_size();
    const int point_block_size = bal_problem->point_block_size();

    double*raw_cameras=bal_problem->mutable_cameras(); //mutable为易变的
    for(int i=0;i<num_cameras;i++)
    {
        VertexCameraBAL*pCamera= dynamic_cast<VertexCameraBAL*>(optimizer->vertex(i)); //这几行不知道拿来干嘛的
        Eigen::VectorXd NewCameraVec=pCamera->estimate();
        memcpy(raw_cameras+i*camera_block_size,NewCameraVec.data(),sizeof(double)*camera_block_size);
    }

    double *raw_points=bal_problem->mutable_points();
    for(int j=0;j<num_points;j++)
    {
        VertexPointBAL*pPoint= dynamic_cast<VertexPointBAL*>(optimizer->vertex(j+num_cameras));
        Eigen::Vector3d NewPointVec=pPoint->estimate();
        memcpy(raw_points+j*point_block_size,NewPointVec.data(),sizeof(double)*point_block_size);
        //void *memcpy(void *dest, const void *src, size_t n);
        // 从源src所指的内存地址的起始位置开始拷贝n个字节到目标dest所指的内存地址的起始位置中
    }

}
void SetSolverOptionsFromFlags(BALProblem*bal_problem,const BundleParams&params,g2o::SparseOptimizer*optimizer) {
    std::unique_ptr<BalBlockSolver::LinearSolverType> linearSolver;
    if (params.linear_solver == "dense_schur") {
        std::unique_ptr<BalBlockSolver::LinearSolverType> linearSolver(
                new g2o::LinearSolverDense<BalBlockSolver::PoseMatrixType>());
    } else if (params.linear_solver == "sparse_schur") {
        std::unique_ptr<BalBlockSolver::LinearSolverType> linearSolver(
                new g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>());
    }

    std::unique_ptr<BalBlockSolver> solver_ptr(new BalBlockSolver(std::move(linearSolver)));

    g2o::OptimizationAlgorithmWithHessian *solver;
    if (params.trust_region_strategy == "levenberg_marquardt") {
        g2o::OptimizationAlgorithmWithHessian *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    } else if (params.trust_region_strategy == "dogleg") {
        g2o::OptimizationAlgorithmWithHessian *solver = new g2o::OptimizationAlgorithmDogleg(std::move(solver_ptr));
    } else {
        std::cout << "please check your trust_region_strategy parameter again.." << std::endl;
        exit(EXIT_FAILURE);
    }

    optimizer->setAlgorithm(solver);
   }

void SolveProblem(const char* filename,const BundleParams&params)
{
    cout<<"filename path:"<<filename<<endl;
    BALProblem bal_problem(filename);

    //show some information
    std::cout<<"bal problem file loaded..."<<std::endl;
    std::cout<<"bal problem have "<<bal_problem.num_cameras()<<"cameras and "<<bal_problem.num_points()<<" points. "<<std::endl;
    std::cout<<"Forming "<<bal_problem.num_observations()<<" observations. "<<std::endl;


    //store the initial 3D cloud points and camera pose..

        bal_problem.WriteToPLYFile(params.initial_ply);


    //add some noise for initial value
    srand(params.random_seed);
    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma,params.translation_sigma,params.point_sigma);

    std::cout<<"Normalization complete..."<<std::endl;

    g2o::SparseOptimizer optimizer;
    SetSolverOptionsFromFlags(&bal_problem,params,&optimizer);
    BuildProblem(&bal_problem,&optimizer,params);

    std::cout<<"begin optimization.."<<std::endl;
    //perform the optimization
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    std::cout<<"optimization complete.."<<std::endl;
    //write the result into a ./ply file
    if(!params.final_ply.empty())
    {
        bal_problem.WriteToPLYFile(params.final_ply);
    }
}

int main(int argc,char**argv)
{
    cout<<"argc: "<<argc<<endl;
    cout<<"argv[1]: "<<argv[1]<<endl;
    cout<<"argv[0]: "<<argv[0]<<endl;
    BundleParams params(argc,argv);
    if(params.input.empty())
    {
        std::cout<<"usage: bundle_adjuster -input<path for dataset>";
        return 1;
    }


    SolveProblem(params.input.c_str(),params);

    return  0;
}



