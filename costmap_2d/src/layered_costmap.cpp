/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using std::vector;

namespace costmap_2d
{
//Costmap2D costmap_;
LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown) :
    costmap_(),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    current_(false),
    minx_(0.0),
    miny_(0.0),
    maxx_(0.0),
    maxy_(0.0),
    bx0_(0),
    bxn_(0),
    by0_(0),
    byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.1)
{
  //就是设置类costmap_2d 的一个成员变量default_value_ 的值
  if (track_unknown)
    costmap_.setDefaultValue(255);
  else
    costmap_.setDefaultValue(0);
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}
/*

函数LayeredCostmap::resizeMap就是给class costmap_2d 的costmap_ 成员的大小
重新做分配。然后根据plugin对每一层的地图调用其父类Costmap2D成员的initial 方法，
实际效果就是将plugin所指向的每一层地图的大小都设置为和LayeredCostmap::costmap_ 
数据成员一样的空间大小。


这个函数在Costmap2DROS动态配置参数的回调函数ReconfigureCB中被调用，
作用是在开启地图更新线程之前，调用Costmap2D的resizeMap函数，
用给定参数重新设置主地图的尺寸、原点、分辨率，
再通过plugin指针调用各层地图的matchSize，使其以上参数和主地图匹配。
646
*/
void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  size_locked_ = size_locked;
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->matchSize();
  }
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  //rolling_window_默认为false，如果开启的话，地图是时刻跟随机器人中心移动的，
  //这里需要根据机器人当前位置和地图大小计算出地图的新原点，设置给主地图。
  // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
  if (rolling_window_)
  {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  if (plugins_.size() == 0)
    return;

  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;

  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    //这个阶段会更新每个Layer的更新区域，这样在每个运行周期内减少了数据拷贝的操作时间。
    //更新边界区域
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    //更新边界区域
   
    
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
    }
  }

  int x0, xn, y0, yn;
  //将得到的bound转换到地图坐标系。这个函数可以防止转换后的坐标超出地图范围。
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);
  //简单的比较，得到主地图的范围，下面会进行判断
  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);
 
  ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);
  //错误，不用继续执行，直接跳出去
  if (xn < x0 || yn < y0)
    return;
  //接下来，调用resetMap，将主地图上bound范围内的cell的cost恢复为
  //默认值（track_unknown：255 / 否则：0），
  //再对每层子地图调用updateCosts函数。
  costmap_.resetMap(x0, y0, xn, yn); //就是复位函数
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
     //函数的第一个参数是指的master map，
     //后面的bounds是对每个plugin自己维护的map的更新界限做设定。
    (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}
//主要的操作是对操作的实时性提供保证，提供是否发生超时的信息。
bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    current_ = current_ && (*plugin)->isCurrent();
  }
  return current_;
}
//inscribed_radius_, circumscribed_radius_ 
//是计算得到的机器人尺寸的内切圆和外切圆半径。
void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace costmap_2d
