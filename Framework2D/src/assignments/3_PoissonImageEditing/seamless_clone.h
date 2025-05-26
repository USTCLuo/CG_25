#pragma once
#include <iostream>
#include <common/image.h>
#include <Eigen/Sparse>
#include "source_image_widget.h"

namespace USTC_CG
{
    class SeamlessClone
    {
        public:
            SeamlessClone() = default;
            virtual ~SeamlessClone() = default;

            // 获取指定坐标的像素值
            int g(int x, int y,int channel);
            double f(int x, int y, int channel, int o_x, int o_y);

            // 主求解函数
            std::shared_ptr<Image> solve();

            // 设置图像数据
            void set_src_img(const std::shared_ptr<Image>& src_img) { src_img_ = src_img; }
            void set_tar_img(const std::shared_ptr<Image>& tar_img) { tar_img_ = tar_img; }
            void set_src_selected_mask(const std::shared_ptr<Image>& src_selected_mask) { src_selected_mask_ = src_selected_mask; }
            void set_offset(int offset_x, int offset_y) { offset_x_ = offset_x; offset_y_ = offset_y; }
            int get_offset_x() {return offset_x_;}
            int get_offset_y() {return offset_y_;}
            void precomputed_A();

        private:
            std::shared_ptr<Image> src_img_; // 源图像
            std::shared_ptr<Image> tar_img_; // 背景图像
            std::shared_ptr<Image> src_selected_mask_; // 选中区域
            int offset_x_, offset_y_; // 偏移量

            std::vector<Eigen::VectorXd> cached_pixels_; // 缓存所有像素值
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_;
            Eigen::SparseMatrix<double> A_;
            bool A_precomputed_ = false;//标记A是否被预分解

            //混合梯度算法
            Eigen::Vector2d compute_gradient(const std::shared_ptr<Image>& img ,int x,int y,int channel);
            Eigen::Vector2d mix_gradient(int x,int y,int channel);
    };
}