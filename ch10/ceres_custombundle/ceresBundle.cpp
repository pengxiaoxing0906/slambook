#include <iostream>
#include <fstream>
#include "ceres/ceres.h"

#include "SnavelyReprojectionError.h"
#include "common/BALProblem.h"
#include "common/BundleParams.h"

using namespace ceres;
using namespace std;


void SolveProblem(const char*filename,const BundleParams& params);
void BuildProblem(BALProblem* bal_problem,Problem*problem,const BundleParams&params);
void SetSolverOptionsFromFlags(BALProblem* bal_problem,const BundleParams&params,Solver::Options*options);
void SetMinimizerOptions(Solver::Options*options,const BundleParams&params);
void SetlinearSolver(Solver::Options*options,const BundleParams&params);
void SetOrdering(BALProblem* bal_problem,Solver::Options*options,const BundleParams&params);

int main(int argc,char**argv)
{
    BundleParams params(argc,argv); //set the parameters here.

    google::InitGoogleLogging(argv[0]);
    if(params.input.empty())
    {
        cout<<"usage:bundle_adjuster -input <path for dataset>";
        return 1;
    }

    SolveProblem(params.input.c_str(),params);
    return 0;
}
void SolveProblem(const char*filename,const BundleParams& params)
{
    BALProblem bal_problem(filename);

    //show some information here..
    cout<<"bal problem file loaded..."<<endl;
    cout<<"bal problem have "<<bal_problem.num_cameras()<<" cameras and "<<bal_problem.num_points()<<" points."<<endl;
    cout<<"forming "<<bal_problem.num_observations()<<" observations. "<<endl;


    //store the initial 3D cloud points and camera pose..
    if(!params.initial_ply.empty())
    {
        bal_problem.WriteToPLYFile(params.initial_ply);
    }
    cout<<"beginning problem..."<<endl;

    //add some noise for the initial value
    srand(params.random_seed);
    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma,params.translation_sigma,params.point_sigma);

    cout<<"Normalization complete..."<<endl;

    Problem problem;
    BuildProblem(&bal_problem,&problem,params); //就是写problem.AddResidualBlock(cost_function,loss_function,camera,point)这个函数

    cout<<"the problem is successfully build.."<<endl;

    Solver::Options options;
    SetSolverOptionsFromFlags(&bal_problem,params,&options);
    options.gradient_tolerance=1e-16;
    options.function_tolerance=1e-16;
    Solver::Summary summary;
    Solve(options,&problem,&summary);

    cout<<summary.FullReport()<<endl;

    //write the result into a .ply file.
    if(!params.final_ply.empty())
    {
        bal_problem.WriteToPLYFile(params.final_ply);
    }

}
void BuildProblem(BALProblem* bal_problem,Problem*problem,const BundleParams&params) {
    const int point_block_size = bal_problem->point_block_size();//point的维度为３
    const int camera_block_size = bal_problem->camera_block_size();//camera的维度为９　R t f k1 k2
    double *points = bal_problem->mutable_points(); //可计算的点???
    double *cameras = bal_problem->mutable_cameras();

    //observations id 2*num_observations long array observations
    //[u_1,u_2...]u_i is two dimensional,the xand y position of the observation
    const double *observations = bal_problem->observations();

    for (int i = 0; i < bal_problem->num_observations(); ++i) {
        //each residual block takes a point and a camera as input and outputs a 2 dimensional residual
        CostFunction *cost_function;
        cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);

        //if enabled use Huber's loss function
        LossFunction *loss_function = params.robustify ? new HuberLoss(1.0) : NULL;

        //each observation corresponds to a pair of a camera and a point which are identfied by camera_index()[i] and point_index()[i] respectively
        double *camera = cameras + camera_block_size * bal_problem->camera_index()[i];
        double *point = points + point_block_size * bal_problem->point_index()[i];

        problem->AddResidualBlock(cost_function, loss_function, camera, point);
    }
}


void SetSolverOptionsFromFlags(BALProblem* bal_problem,const BundleParams&params,Solver::Options*options)
{
        SetMinimizerOptions(options,params);
        SetlinearSolver(options,params);
        SetOrdering(bal_problem,options,params);


}
void SetMinimizerOptions(Solver::Options*options,const BundleParams&params)
{
    options->max_num_iterations=params.num_iterations;
    options->minimizer_progress_to_stdout=true;//输出到cout
    options->num_threads=params.num_threads;//这代表啥意思
    CHECK(StringToTrustRegionStrategyType(params.trust_region_strategy,&options->trust_region_strategy_type));


}

void SetlinearSolver(Solver::Options*options,const BundleParams&params)
{
    CHECK(ceres::StringToLinearSolverType(params.linear_solver,&options->linear_solver_type));
    CHECK(ceres::StringToSparseLinearAlgebraLibraryType(params.sparse_linear_algebra_library,&options->sparse_linear_algebra_library_type));
    CHECK(ceres::StringToDenseLinearAlgebraLibraryType(params.dense_linear_algebra_library,&options->dense_linear_algebra_library_type));


}
void SetOrdering(BALProblem* bal_problem,Solver::Options*options,const BundleParams&params)
{
    const int num_points=bal_problem->num_points();
    const int point_block_size=bal_problem->point_block_size();
    double*points=bal_problem->mutable_points();

    const int num_cameras=bal_problem->num_cameras();
    const int camera_block_size=bal_problem->camera_block_size();
    double*cameras=bal_problem->mutable_cameras();

    if(params.ordering=="automatic")
        return;

    ceres::ParameterBlockOrdering*ordering=new ceres::ParameterBlockOrdering;

    //the points come before the cameras
    for(int i=0;i<num_points;++i)
    {
        ordering->AddElementToGroup(points+point_block_size*i,0);//这个函数的功能是对变量进行编号从而定义消元顺序　优先消元编号最小的编号
    }

    for(int i=0;i<num_cameras;++i)
    {
        ordering->AddElementToGroup(cameras+camera_block_size*i,1);
    }

    options->linear_solver_ordering.reset(ordering);

}