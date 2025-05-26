#include "polygon.h"

#include <imgui.h>

namespace USTC_CG
{
    void Polygon::draw(const Config& config) const
    {
        if (control_points_.size() < 2) return;
    
        // 绘制所有线段
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        for (size_t i = 0; i < control_points_.size(); ++i) 
        {
            const auto& p1 = control_points_[i];
            const auto& p2 = control_points_[(i+1) % control_points_.size()];
            draw_list->AddLine(
                ImVec2(config.bias[0] + p1.first, config.bias[1] + p1.second), 
                ImVec2(config.bias[0] + p2.first, config.bias[1] + p2.second),
                IM_COL32(
                    config.line_color[0],
                    config.line_color[1],
                    config.line_color[2],
                    config.line_color[3]),
                config.line_thickness
            );
        }
    
        // 绘制控制点
        for (const auto& p : control_points_) 
        {
            draw_list->AddCircleFilled(
                ImVec2(config.bias[0] + p.first, config.bias[1] + p.second), 
                3.0f, 
                IM_COL32(
                    config.line_color[0],
                    config.line_color[1],
                    config.line_color[2],
                    config.line_color[3])
            );
        }
    }
}  // namespace USTC_CG