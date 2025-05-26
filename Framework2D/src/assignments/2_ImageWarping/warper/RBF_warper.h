// HW2_TODO: Implement the RBFWarper class
#pragma once

#include "warper.h"
#include <vector>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;
namespace USTC_CG
{
class RBFWarper : public Warper
{
   public:
    RBFWarper();
    virtual ~RBFWarper() = default;
    // HW2_TODO: Implement the warp(...) function with RBF interpolation
    std::pair<double,double> warp(double x, double y) override;
    // HW2_TODO: other functions or variables if you need
    void setControlPoints(const std::vector<Vector2f>& start_points,
        const std::vector<Vector2f>& end_points);
    void clearControlPoints()
    {
        sourcePoints.clear(); 
        alphas.clear();       
        A.setIdentity();       
        b.setZero();
    }
    std::pair<double, double> inverse_warp(double target_x, double target_y, int max_iter, float tol);
    private:
     float radialBasisFunction(float d, float r) const;

     Matrix2f A;//仿射变换矩阵A
     Vector2f b;//向量b
     std::vector<Vector2f> alphas;//径向基函数权重
     std::vector<Vector2f> sourcePoints;//控制点
     std::vector<float> r;
     
};
}  // namespace USTC_CG