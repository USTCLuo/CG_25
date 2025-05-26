#include "canvas_widget.h"
#include "shapes/shape.h"

#include <cmath>
#include <iostream>

#include "imgui.h"
#include "shapes/line.h"
#include "shapes/rect.h"
#include "shapes/ellipse.h"
#include "shapes/polygon.h"
#include "shapes/freehand.h"

namespace USTC_CG
{
    Shape::Config s;
    float thickset=2.0f;
    int colorset[4]={255,0,0,255};
void Canvas::draw()
{
    draw_background();
    // HW1_TODO: more interaction events
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        mouse_click_event();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        mouse_right_click_event();
    mouse_move_event();
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        mouse_release_event();

    draw_shapes();
}

void Canvas::set_attributes(const ImVec2& min, const ImVec2& size)
{
    canvas_min_ = min;
    canvas_size_ = size;
    canvas_minimal_size_ = size;
    canvas_max_ =
        ImVec2(canvas_min_.x + canvas_size_.x, canvas_min_.y + canvas_size_.y);
}

void Canvas::show_background(bool flag)
{
    show_background_ = flag;
}

void Canvas::set_default()
{
    draw_status_ = false;
    shape_type_ = kDefault;
}

void Canvas::set_line()
{
    draw_status_ = false;
    shape_type_ = kLine;
}

void Canvas::set_rect()
{
    draw_status_ = false;
    shape_type_ = kRect;
}

// HW1_TODO: more shape types, implements

void Canvas::set_ellipse()
{
    draw_status_ = false;
    shape_type_ = kEllipse;
}
void Canvas::set_polygon()
{
    draw_status_ = false;
    shape_type_ = kPolygon;
}
void Canvas::set_freehand()
{
    draw_status_ = false;
    shape_type_ = kFreehand;
}
void Canvas::set_thickness()
{   
    std::cin >> thickset;
}
void Canvas::set_color()
{
    for(int i=0;i<4;i++)
    {
        std::cin >> colorset[i];
    }
}

void Canvas::clear_shape_list()
{
    shape_list_.clear();
}

void Canvas::draw_background()
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    if (show_background_)
    {
        // Draw background recrangle
        draw_list->AddRectFilled(canvas_min_, canvas_max_, background_color_);
        // Draw background border
        draw_list->AddRect(canvas_min_, canvas_max_, border_color_);
    }
    /// Invisible button over the canvas to capture mouse interactions.
    ImGui::SetCursorScreenPos(canvas_min_);
    ImGui::InvisibleButton(
        label_.c_str(), canvas_size_, ImGuiButtonFlags_MouseButtonLeft);
    // Record the current status of the invisible button
    is_hovered_ = ImGui::IsItemHovered();
    is_active_ = ImGui::IsItemActive();
}

void Canvas::draw_shapes()
{  
    s = { .bias = { canvas_min_.x, canvas_min_.y } };
    s.line_thickness = thickset;
    for(int i=0;i<4;i++)
    {
        s.line_color[i] = colorset[i];
    }
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // ClipRect can hide the drawing content outside of the rectangular area
    draw_list->PushClipRect(canvas_min_, canvas_max_, true);
    for (const auto& shape : shape_list_)
    {
        shape->draw(s);
    }
    if (draw_status_ && current_shape_)
    {
        current_shape_->draw(s);
    }
    draw_list->PopClipRect();
}

void Canvas::mouse_click_event()
{
    // HW1_TODO: Drawing rule for more primitives
    if (!draw_status_)
    {   
        draw_status_ = true;
        start_point_ = end_point_ = mouse_pos_in_canvas();
        switch (shape_type_)
        {
            case USTC_CG::Canvas::kDefault:
            {
                break;
            }
            case USTC_CG::Canvas::kLine:
            {
                current_shape_ = std::make_shared<Line>(
                    start_point_.x, start_point_.y, end_point_.x, end_point_.y);
                break;
            }
            case USTC_CG::Canvas::kRect:
            {
                current_shape_ = std::make_shared<Rect>(
                    start_point_.x, start_point_.y, end_point_.x, end_point_.y);
                break;
            }
            // HW1_TODO: case USTC_CG::Canvas::kEllipse:
            case USTC_CG::Canvas::kEllipse:
            {
                current_shape_ = std::make_shared<Ellipse>(
                    start_point_.x, start_point_.y, end_point_.x, end_point_.y
                );
                break;
            }
            case USTC_CG::Canvas::kPolygon:
            {
                current_shape_ = std::make_shared<Polygon>();
                auto polygon = std::dynamic_pointer_cast<Polygon>(current_shape_);
                polygon->add_point(start_point_.x, start_point_.y); // 固定点
                polygon->add_point(start_point_.x, start_point_.y); // 动态点
                draw_status_ = true;
                break;
            }
            case USTC_CG::Canvas::kFreehand:
            {
                current_shape_ = std::make_shared<Freehand>();
                auto freehand = std::dynamic_pointer_cast<Freehand>(current_shape_);
                draw_status_ = true;
                break;
            }
            default: break;
        }
    }
    else 
    {
        // 只处理多边形的顶点添加
        if (shape_type_ == kPolygon) 
        {
            auto polygon = std::dynamic_pointer_cast<Polygon>(current_shape_);
            if (polygon) 
            {
                polygon->add_point(end_point_.x, end_point_.y);
            }
        }
    }
}

void Canvas::mouse_move_event()
{
    // HW1_TODO: Drawing rule for more primitives
    if (draw_status_)
    {
        end_point_ = mouse_pos_in_canvas();
        if (current_shape_)
        {
            if(shape_type_ == kPolygon){
                auto polygon = std::dynamic_pointer_cast<Polygon>(current_shape_);
                if(polygon && !polygon->get_points().empty())
                {
                    auto& points = polygon->get_points();
                    points.back().first = end_point_.x;
                    points.back().second = end_point_.y;
                }
            }
            else if(shape_type_ == kFreehand)
            {
                auto freehand = std::dynamic_pointer_cast<Freehand>(current_shape_);
                freehand->add_point(start_point_.x, start_point_.y); // 动态点
                current_shape_->update(end_point_.x, end_point_.y);
                if(freehand && !freehand->get_points().empty())
                {
                    auto& points = freehand->get_points();
                    points.back().first = end_point_.x;
                    points.back().second = end_point_.y;
                }
            }
            else
            {
                current_shape_->update(end_point_.x, end_point_.y);
            }
        }
    }
}

void Canvas::mouse_right_click_event()
{
    if (shape_type_ == kPolygon && current_shape_) 
    {
        auto polygon = std::dynamic_pointer_cast<Polygon>(current_shape_);
        if (polygon && polygon->get_points().size() >= 3) 
        {
            // 移除最后的动态顶点
            auto& points = polygon->get_points();
            points.pop_back();
            // 完成绘制
            shape_list_.push_back(current_shape_);
            current_shape_.reset();
            draw_status_ = false;
        }
    }
}
void Canvas::mouse_release_event()
{
    if (draw_status_)
    {
        // 只处理非多边形图形的结束
        if (shape_type_ != kPolygon)
        {
            end_point_ = mouse_pos_in_canvas();
            if (current_shape_)
            {
                // 更新终点坐标
                current_shape_->update(end_point_.x, end_point_.y);
                // 将当前图形添加到列表
                shape_list_.push_back(current_shape_);
                // 重置当前图形和绘制状态
                current_shape_.reset();
                draw_status_ = false;
            }
        }
    }
}

ImVec2 Canvas::mouse_pos_in_canvas() const
{
    ImGuiIO& io = ImGui::GetIO();
    const ImVec2 mouse_pos_in_canvas(
        io.MousePos.x - canvas_min_.x, io.MousePos.y - canvas_min_.y);
    return mouse_pos_in_canvas;
}
}  // namespace USTC_CG