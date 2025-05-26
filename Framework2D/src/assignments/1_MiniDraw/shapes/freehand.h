#pragma once

#include "shape.h"
#include<vector>
#include<utility>

namespace USTC_CG
{
class Freehand : public Shape
{
   public:
    using Point = std::pair<float,float>;
    Freehand() = default;

    explicit Freehand(const std::vector<Point>& points )
        :
          control_points_(points),
          count_(false) 
        {
        }
    Freehand(
        bool count,
        float init_point_x,
        float init_point_y)
        : count_(count)
    {
      control_points_.emplace_back(init_point_x,init_point_y);
    }

    virtual ~Freehand() = default;

    void draw(const Config& config) const override;

    // Overrides Shape's update function to adjust the end point during
    // interaction
    void update(float x, float y) override
    {
      if(!control_points_.empty())
      {
        control_points_.back()={x,y};
      }
    }
    std::vector<Point>& get_points()
    {
      return control_points_;
    }
    void add_point(float x,float y)
    {
      control_points_.emplace_back(x,y);
    }
   private:
    std::vector<Point> control_points_;
    bool count_=false;
};
}  // namespace USTC_CG
