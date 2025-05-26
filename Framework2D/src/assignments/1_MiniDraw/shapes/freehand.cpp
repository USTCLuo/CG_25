#include "freehand.h"

#include <imgui.h>

namespace USTC_CG
{
    void Freehand::draw(const Config& config) const
    {
        if (control_points_.size() < 2) return;
    
        // 绘制所有线段
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        for (size_t i = 0; i < control_points_.size()-1; ++i) 
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
    }
}  // namespace USTC_CG