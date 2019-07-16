#ifndef SnavelyReprojection_H
#define SnavelyReprojection_H

#include <iostream>
#include "ceres/ceres.h"


#include "common/tools/rotation.h"
#include "common/projection.h"

class SnavelyReprojectionError
{
private:
    double observed_x;
    double observed_y;
public:
    SnavelyReprojectionError(double observation_x, double observation_y):observed_x(observation_x),observed_y(observation_y){}
    template <typename T>
    bool operator()(const T*const camera,const T* const point,T*residuals)const{
        //camera[0,1,2] are the angle-axis rotation
        T prediction[2];
        residuals[0]=prediction[0]-T(observed_x);
        residuals[1]=prediction[1]-T(observed_y);

        return  true;
    }
    static ceres::CostFunction* Create(const double observed_x,const double observed_y){
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError,2,9,3>(new SnavelyReprojectionError(observed_x,observed_y)));
    }


};
#endif // SnavelyReprojection.h

