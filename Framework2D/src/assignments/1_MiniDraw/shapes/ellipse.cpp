#include "ellipse.h"

#include <imgui.h>

namespace USTC_CG
{
// Draw the line using ImGui
void Ellipse::draw(const Config& config) const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    draw_list->AddEllipse(
        ImVec2(
            config.bias[0] + (start_point_x_+end_point_x_)/2,config.bias[1] + (start_point_y_+end_point_y_)/2),
        ImVec2((end_point_x_ - start_point_x_)/2,(end_point_y_ - start_point_y_)/2),
        IM_COL32(
            config.line_color[0],
            config.line_color[1],
            config.line_color[2],
            config.line_color[3]),
        0.0f,
        32,
        config.line_thickness);
}

void Ellipse::update(float x, float y)
{
    end_point_x_ = x;
    end_point_y_ = y;
}
}  // namespace USTC_CG