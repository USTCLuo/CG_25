#include "seamless_clone.h"
#include <Eigen/Sparse>
#include <iostream>
#include <vector>

namespace USTC_CG
{

    // 获取指定坐标的像素值
    int SeamlessClone::g(int x, int y, int channel)
    {
        // 检查坐标是否在图像范围内
        if (x < 0 || x >= src_img_->width() || y < 0 || y >= src_img_->height())
        {
            return 0; // 返回 0 表示无效的像素值
        }

        // 从缓存中获取像素值
        int idx = y * src_img_->width() + x;
        return cached_pixels_[idx](channel);
    }

    // 处理选中区域的边界点
    double SeamlessClone::f(int x, int y, int channel, int o_x, int o_y)
    {
        double result = 0.0;
        Eigen::Vector2d grad = mix_gradient(x, y, channel);
        if (y - 1 >= 0 && src_selected_mask_->get_pixel(x, y - 1)[0] == 0)
            result += tar_img_->get_pixel(x + o_x, y - 1 + o_y)[channel] + grad[1];
        if (y + 1 < src_img_->height() && src_selected_mask_->get_pixel(x, y + 1)[0] == 0)
            result += tar_img_->get_pixel(x + o_x, y + 1 + o_y)[channel] - grad[1];
        if (x - 1 >= 0 && src_selected_mask_->get_pixel(x - 1, y)[0] == 0)
            result += tar_img_->get_pixel(x - 1 + o_x, y + o_y)[channel] + grad[0];
        if (x + 1 < src_img_->width() && src_selected_mask_->get_pixel(x + 1, y)[0] == 0)
            result += tar_img_->get_pixel(x + 1 + o_x, y + o_y)[channel] - grad[0];
        return result;
    }

    // 计算图像的梯度
    Eigen::Vector2d SeamlessClone::compute_gradient(const std::shared_ptr<Image>& img, int x, int y, int channel)
    {
        double dx = 0.0, dy = 0.0;
        if (x + 1 < img->width() && x - 1 >= 0)
            dx = img->get_pixel(x + 1, y)[channel] - img->get_pixel(x - 1, y)[channel];
        if (y + 1 < img->height() && y - 1 >= 0)
            dy = img->get_pixel(x, y + 1)[channel] - img->get_pixel(x, y - 1)[channel];
        return Eigen::Vector2d(dx, dy);
    }

    // 混合梯度
    Eigen::Vector2d SeamlessClone::mix_gradient(int x, int y, int channel)
    {
        Eigen::Vector2d grad_src = compute_gradient(src_img_, x, y, channel);
        Eigen::Vector2d grad_tar = compute_gradient(tar_img_, x + offset_x_, y + offset_y_, channel);

        if (grad_src.norm() > grad_tar.norm())
        {
            return grad_src;
        }
        else
        {
            return grad_tar;
        }
    }

    // 预分解稀疏矩阵 A
    void SeamlessClone::precomputed_A()
    {
        int W = src_img_->width();
        int H = src_img_->height();

        // 计算选中区域的像素数量 K
        int K = 0;
        std::vector<int> selected_pixels;
        for (int y = 0; y < H; ++y)
        {
            for (int x = 0; x < W; ++x)
            {
                if (src_selected_mask_->get_pixel(x, y)[0] > 0) // 选中区域
                {
                    selected_pixels.push_back(y * W + x);
                    ++K;
                }
            }
        }

        A_.resize(K, K);
        std::vector<Eigen::Triplet<double>> triplet_list;

        // 构建稀疏矩阵 A
        for (int i = 0; i < K; ++i)
        {
            int idx = selected_pixels[i];
            int x = idx % W;
            int y = idx / W;

            // 填写稀疏矩阵 A 的系数
            triplet_list.push_back(Eigen::Triplet<double>(i, i, 4.0));
            if (y - 1 >= 0 && src_selected_mask_->get_pixel(x, y - 1)[0] > 0)
                triplet_list.push_back(Eigen::Triplet<double>(i, std::find(selected_pixels.begin(), selected_pixels.end(), (y - 1) * W + x) - selected_pixels.begin(), -1.0));
            if (y + 1 < H && src_selected_mask_->get_pixel(x, y + 1)[0] > 0)
                triplet_list.push_back(Eigen::Triplet<double>(i, std::find(selected_pixels.begin(), selected_pixels.end(), (y + 1) * W + x) - selected_pixels.begin(), -1.0));
            if (x - 1 >= 0 && src_selected_mask_->get_pixel(x - 1, y)[0] > 0)
                triplet_list.push_back(Eigen::Triplet<double>(i, std::find(selected_pixels.begin(), selected_pixels.end(), y * W + (x - 1)) - selected_pixels.begin(), -1.0));
            if (x + 1 < W && src_selected_mask_->get_pixel(x + 1, y)[0] > 0)
                triplet_list.push_back(Eigen::Triplet<double>(i, std::find(selected_pixels.begin(), selected_pixels.end(), y * W + (x + 1)) - selected_pixels.begin(), -1.0));
        }

        // 构建稀疏矩阵 A
        A_.setFromTriplets(triplet_list.begin(), triplet_list.end());

        // 预分解稀疏矩阵 A
        solver_.compute(A_);
        A_precomputed_ = true;
    }

    // 主求解函数
    std::shared_ptr<Image> SeamlessClone::solve()
    {
        int W = src_img_->width();
        int H = src_img_->height();

        // 缓存 src_img_ 的所有像素值
        cached_pixels_.resize(src_img_->width() * src_img_->height());
        for (int y = 0; y < src_img_->height(); ++y)
        {
            for (int x = 0; x < src_img_->width(); ++x)
            {
                auto pixel_value = src_img_->get_pixel(x, y);
                Eigen::VectorXd pixel_vector(3); // 假设图像是 RGB 彩色图像
                pixel_vector(0) = static_cast<double>(pixel_value[0]); // R 通道
                pixel_vector(1) = static_cast<double>(pixel_value[1]); // G 通道
                pixel_vector(2) = static_cast<double>(pixel_value[2]); // B 通道
                cached_pixels_[y * src_img_->width() + x] = pixel_vector;
            }
        }

        auto result_image = std::make_shared<Image>(W, H, 3);

        // 计算选中区域的像素数量 K
        int K = 0;
        std::vector<int> selected_pixels;
        for (int y = 0; y < H; ++y)
        {
            for (int x = 0; x < W; ++x)
            {
                if (src_selected_mask_->get_pixel(x, y)[0] > 0) // 选中区域
                {
                    selected_pixels.push_back(y * W + x);
                    ++K;
                }
            }
        }

        // 对每个通道分别求解
        for (int channel = 0; channel < 3; ++channel)
        {
            Eigen::VectorXd B(K);

            // 构建向量 B
            for (int i = 0; i < K; ++i)
            {
                int idx = selected_pixels[i];
                int x = idx % W;
                int y = idx / W;

                // 填写向量 B 的系数
                B(i) = 4.0 * g(x, y, channel) -
                        (y - 1 >= 0 ? g(x, y - 1, channel) : 0) -
                        (y + 1 < H ? g(x, y + 1, channel) : 0) -
                        (x - 1 >= 0 ? g(x - 1, y, channel) : 0) -
                        (x + 1 < W ? g(x + 1, y, channel) : 0) +
                        f(x, y, channel, offset_x_, offset_y_);
            }

            Eigen::VectorXd r = solver_.solve(B);

            // 将结果写入结果图像
            for (int i = 0; i < K; ++i)
            {
                int idx = selected_pixels[i];
                int x = idx % W;
                int y = idx / W;
                auto pixel_value = result_image->get_pixel(x, y);
                pixel_value[channel] = static_cast<unsigned char>(std::clamp(r(i), 0.0, 255.0));
                result_image->set_pixel(x, y, pixel_value);
            }
        }

        return result_image;
    }

}

