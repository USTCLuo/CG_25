#pragma once

#include "warper\IDW_warper.h"
#include "warper\RBF_warper.h"
#include <Annoy/annoylib.h>
#include <Annoy/kissrandom.h>
#include <Annoy/mman.h>
#include <vector>
#include <cmath>
#include<iostream>

#include "common/image_widget.h"

using namespace Annoy;
namespace USTC_CG
{
// Image component for warping and other functions
class WarpingWidget : public ImageWidget
{
   public:
    explicit WarpingWidget(
        const std::string& label,
        const std::string& filename);
    virtual ~WarpingWidget() noexcept = default;

    void draw() override;

    // Simple edit functions
    void invert();
    void mirror(bool is_horizontal, bool is_vertical);
    void gray_scale();
    void warping();
    void restore();

    // Enumeration for supported warping types.
    // HW2_TODO: more warping types.
    enum WarpingType
    {
        kDefault = 0,
        kFisheye = 1,
        kIDW = 2,
        kRBF = 3,
    };
    // Warping type setters.
    void set_default();
    void set_fisheye();
    void set_IDW();
    void set_RBF();

    // Point selecting interaction
    void enable_selecting(bool flag);
    void select_points();
    void init_selections();

    std::vector<unsigned char> get_nearby_pixels(float x, float y, int k, 
        AnnoyIndex<int, float, Euclidean, Kiss32Random, AnnoyIndexSingleThreadedBuildPolicy>& index, const Image& image, float sigma)
    {
        int width = image.width();
        int height = image.height();

        // 目标点坐标
        float point[2] = {x, y};

        // 查找最近的 k 个邻近像素
        std::vector<int> nearest_ids(0);
        std::vector<float> distances(0);
        index.get_nns_by_vector(point, k, -1, &nearest_ids, &distances);

        // 计算加权平均值
        std::vector<float> weighted_sum(3, 0.0f);  // 存储加权和的 R, G, B 值
        float total_weight = 0.0f;

        for (int i = 0; i < k; ++i) {
            int id = nearest_ids[i];
            int px = id % width;  // 计算像素的 x 坐标
            int py = id / width;  // 计算像素的 y 坐标

            if(px >= 0 && px < width && py >= 0 && py < height)
            {
                // 获取像素值
                auto pixel = image.get_pixel(px, py);

                // 计算权重（距离越近，权重越大）
                float distance = distances[i];
                float weight = std::exp(-distance / (2.0f * sigma * sigma));
                total_weight += weight;

                // 累加加权和
                for (int c = 0; c < 3; ++c) {
                    weighted_sum[c] += weight * pixel[c];
                }
            }
        }

        // 计算加权平均值
        std::vector<unsigned char> result(3);
        if(total_weight > 1e-6f)
        {
            for (int c = 0; c < 3; ++c) {
                result[c] = static_cast<unsigned char>(weighted_sum[c] / total_weight);
            }
        }
        else
        {
            result = image.get_pixel(static_cast<int>(x), static_cast<int>(y));
        }

        return result;
    }
    // 构建 Annoy 索引
    void build_annoy_index(AnnoyIndex<int, float, Euclidean, Kiss32Random,AnnoyIndexSingleThreadedBuildPolicy>& index, const Image& image) {
        int width = image.width();
        int height = image.height();

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int id = y * width + x;  // 像素的唯一 ID
                float point[2] = {static_cast<float>(x), static_cast<float>(y)};  // 像素坐标
                index.add_item(id, point);  // 添加到索引
            }
        }

        index.build(10);  // 构建索引
    }
        
   private:
    // Store the original image data
    std::shared_ptr<Image> back_up_;
    // The selected point couples for image warping
    std::vector<ImVec2> start_points_, end_points_;

    ImVec2 start_, end_;
    bool flag_enable_selecting_points_ = false;
    bool draw_status_ = false;
    WarpingType warping_type_;

   private:
    // A simple "fish-eye" warping function
    std::pair<int, int> fisheye_warping(int x, int y, int width, int height);
    AnnoyIndex<int, float, Euclidean, Kiss32Random,
    AnnoyIndexSingleThreadedBuildPolicy> annoy_index_;
    USTC_CG::IDWWarper idw_warper_;
    USTC_CG::RBFWarper rbf_warper_;
};

}  // namespace USTC_CG