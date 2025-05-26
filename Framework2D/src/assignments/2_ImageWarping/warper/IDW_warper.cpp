#include "IDW_warper.h"

#include <cmath>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;
namespace USTC_CG
{ 
    std::pair<double,double> IDWWarper::warp(double x,double y)
    {
        if(control_points_.empty())
        {
            return {x,y};
        }

        double total_weight = 0.0f;
        double dx_total = 0.0f;
        double dy_total = 0.0f;
        constexpr double epsilon = 1e-10;

        for(const auto& control_point :control_points_)
        {
            double delta_x = x-control_point.src_x;
            double delta_y = y-control_point.src_y;
            double d = delta_x*delta_x + delta_y*delta_y;
            if(d < epsilon)
            {
                return {control_point.target_x, control_point.target_y};
            }
            double weight = 1.0/std::pow(d,mu_);
            total_weight+=weight;

            double cp_dx = control_point.target_x - control_point.src_x;
            double cp_dy = control_point.target_y - control_point.src_y;

            dx_total += cp_dx * weight;
            dy_total += cp_dy * weight;
        }
        double dx_avg = dx_total / total_weight;
        double dy_avg = dy_total / total_weight;

        return {x + dx_avg, y + dy_avg};
        
    }
    std::pair<double, double> IDWWarper::inverse_warp(double target_x, double target_y, int max_iter = 10, float tol = 1e-4) 
    {
        Vector2f target(target_x, target_y);
        Vector2f guess = target; // 初始猜测：假设目标坐标与源坐标相同
        
        for (int iter = 0; iter < max_iter; ++iter) 
        {
            // Step 1: 使用当前猜测值计算正向变形
            auto [warped_x, warped_y] = warp(guess.x(), guess.y());
            Vector2f warped_point(warped_x, warped_y);
            
            // Step 2: 计算残差（变形后坐标与目标坐标的差距）
            Vector2f residual = target - warped_point;
            if (residual.norm() < tol) break; // 收敛条件
            
            // Step 3: 近似雅可比矩阵（数值差分法）
            float eps = 1e-10;
            auto [dx_x, dx_y] = warp(guess.x() + eps, guess.y());
            auto [dy_x, dy_y] = warp(guess.x(), guess.y() + eps);
            
            Matrix2f J;
            J(0,0) = (dx_x - warped_x) / eps; // ∂x'/∂x
            J(0,1) = (dy_x - warped_x) / eps; // ∂x'/∂y
            J(1,0) = (dx_y - warped_y) / eps; // ∂y'/∂x
            J(1,1) = (dy_y - warped_y) / eps; // ∂y'/∂y
            
            // Step 4: 更新猜测值（牛顿迭代）
            if (J.determinant() != 0) 
            {
                guess += J.inverse() * residual;
            }
            else 
            {
                // 雅可比矩阵奇异，使用简单梯度下降
                guess += 0.1 * residual;
            }
        }
        
        return {guess.x(), guess.y()};
    }




}