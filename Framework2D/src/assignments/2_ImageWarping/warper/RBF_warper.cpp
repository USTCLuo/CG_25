#include "RBF_warper.h"
#include<cmath>
#include <iostream>
#include<vector>

using namespace Eigen;
using namespace std;
namespace USTC_CG
{
    RBFWarper::RBFWarper() {
        A.setIdentity();  // 初始化为单位矩阵
        b.setZero();      // 初始化为零向量
    }
    void RBFWarper::setControlPoints(const vector<Vector2f>& start_points,
    const vector<Vector2f>& end_points)
    {
        sourcePoints = start_points;
        int n = sourcePoints.size();
        r.resize(n);
        alphas.resize(n);
        
        for(int i = 0;i < n;i++)
        {
            float minDist = numeric_limits<float>::max();
            for(int j = 0; j<n;j++)
            {
                if(i!=j)
                {
                    float dist = (sourcePoints[i] - sourcePoints[j]).norm();
                    if(dist< minDist) minDist = dist;
                }
            }
            r[i] = minDist;
        }
        Vector2f p1,p2,q1,q2,dp,dq;
        float scale,angle;
        switch (n)
        {
            case 1:
                A.setIdentity();
                b = end_points[0]-sourcePoints[0];
                alphas[0].setZero();
                break;
            case 2:
                p1 = sourcePoints[0];
                p2 = sourcePoints[1];
                q1 = end_points[0];
                q2 = end_points[1];
            
                dp = p2 - p1;
                dq = q2 - q1;
            
                scale = dq.norm() / dp.norm();  // 缩放因子
                angle = atan2(dq.y(), dq.x()) - atan2(dp.y(), dp.x());  // 旋转角度
            
                A << scale * cos(angle), -scale * sin(angle),
                    scale * sin(angle),  scale * cos(angle);  // 旋转+缩放矩阵
                b = q1 - A * p1;  // 平移向量
                break;
            default:
                MatrixXf X(n, 3);
                MatrixXf Y(n, 2);
                for (int i = 0; i < n; i++) {
                    X(i, 0) = sourcePoints[i].x();
                    X(i, 1) = sourcePoints[i].y();
                    X(i, 2) = 1;  // 仿射变换的常数项
                    Y.row(i) = end_points[i];
                }

                // 求解仿射变换参数 [A | b]
                MatrixXf affineParams = X.colPivHouseholderQr().solve(Y);
                A = affineParams.block(0, 0, 2, 2);  // 提取 A
                b = affineParams.row(2).transpose(); // 提取 b
        }

        
        
            
        MatrixXf G(n, n);
        MatrixXf F(n, 2);

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                float d = (sourcePoints[i] - sourcePoints[j]).norm();
                G(i, j) = radialBasisFunction(d, r[i]);
            }
            F.row(i) = end_points[i] - (A * sourcePoints[i] + b);
        }

        //求解权重 alpha_i
        MatrixXf alphaMatrix = G.colPivHouseholderQr().solve(F);
        for (int i = 0; i < n; i++) {
            alphas[i] = alphaMatrix.row(i).transpose();
        }
        
    }

    float RBFWarper::radialBasisFunction(float d,float r) const
    {
        return sqrt(d*d+r*r);
    }
    std::pair<double ,double> RBFWarper::warp(double x,double y)
    {
        Vector2f point(x,y);
        Vector2f newPoint = A*point+b;
        
        float d, rr;

        for(size_t i =0 ;i<sourcePoints.size();i++)
        {
            d = (point - sourcePoints[i]).norm();
            rr = r[i];
            newPoint += alphas[i] * radialBasisFunction(d,rr);
            
        }
        return {newPoint.x(),newPoint.y()};
    }
    std::pair<double, double> RBFWarper::inverse_warp(double target_x, double target_y, int max_iter = 10, float tol = 1e-4) 
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