// HW2_TODO: Implement the IDWWarper class
#pragma once

#include "warper.h"
#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace USTC_CG
{
class IDWWarper : public Warper
{
   public:
    IDWWarper() = default;
    virtual ~IDWWarper() = default;
    // HW2_TODO: Implement the warp(...) function with IDW interpolation
    std::pair<double,double> warp(double x,double y) override;
    // HW2_TODO: other functions or variables if you need
    void add_control_point(double src_x, double src_y, double target_x, double target_y)
    {
        control_points_.push_back({src_x, src_y, target_x, target_y});
    }

    void set(double mu) 
    {
        mu_=mu;
    }

    void clear_control_points()
    {
        control_points_.clear();
    }
    std::pair<double, double> inverse_warp(double target_x, double target_y, int max_iter, float tol);
    private:
    struct ControlPoint
    {
        double src_x, src_y; // x,y
        double target_x, target_y;// x' , y'
    };
    std::vector<ControlPoint> control_points_;//store all points

    double mu_ = 2.0f;
};
}  // namespace USTC_CG