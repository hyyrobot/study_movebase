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
#include <algorithm>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

InflationLayer::InflationLayer()
  : inflation_radius_(0)
  , weight_(0)
  , inflate_unknown_(false)
  , cell_inflation_radius_(0)
  , cached_cell_inflation_radius_(0)
  , dsrv_(NULL)
  , seen_(NULL)
  , cached_costs_(NULL)
  , cached_distances_(NULL)
  , last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
{
  //创建一个递归锁的线程
  inflation_access_ = new boost::recursive_mutex();
}
//主要调用matchsize
void InflationLayer::onInitialize()
{
  {
    //启用这个线程，将这个锁锁上
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    if (seen_)
      delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
    need_reinflation_ = false;

    dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb = boost::bind(
        &InflationLayer::reconfigureCB, this, _1, _2);

    if (dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
  }
    /* 由于InflationLayer没有继承Costmap2D，
    所以它和静态地图与障碍地图这两层不同，它没有属于自己的栅格地图要维护，
    所以matchSize函数自然不需要根据主地图的参数来调节本层地图 */
  matchSize();
}

void InflationLayer::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level)
{
  setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

  if (enabled_ != config.enabled || inflate_unknown_ != config.inflate_unknown) {
    enabled_ = config.enabled;
    inflate_unknown_ = config.inflate_unknown;
    need_reinflation_ = true;
  }
}
//主要完成两个参考矩阵的填充
void InflationLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  //得到主地图上的cost
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  //获得主地图上的分辨率
  resolution_ = costmap->getResolution();
  //这个函数可以把global系以米为单位的长度转换成以cell为单位的距离，
  //故获得了地图上的膨胀参数cell_inflation_radius_
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  //computeCaches函数，完成两个“参考矩阵”的填充
  computeCaches();

  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if (seen_)
    delete[] seen_;
  seen_size_ = size_x * size_y;
  seen_ = new bool[seen_size_];
}


  //need_reinflation_默认false，更新bound这里和其他两层的主要区别是，
  //膨胀层在传入的bound的值的基础上，通过inflation_radius_再次扩张。
void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (need_reinflation_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
  else
  {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void InflationLayer::onFootprintChanged()
{
  //得到内切圆半径
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  //得到外切圆半径
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  //更新参考矩阵
  computeCaches();
  need_reinflation_ = true;

  ROS_DEBUG("InflationLayer::onFootprintChanged(): num footprint points: %lu,"
            " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
            layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  if (!enabled_ || (cell_inflation_radius_ == 0))
    return;

  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  ROS_ASSERT_MSG(inflation_cells_.empty(), "The inflation list must be empty at the beginning of inflation");
  //用指针master_array指向主地图，
  unsigned char* master_array = master_grid.getCharMap();
  //并获取主地图的尺寸，
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
  //确认seen_数组被正确设置，就是将seen_设置成new bool[size_x * size_y];
  if (seen_ == NULL) {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  else if (seen_size_ != size_x * size_y)
  {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  //把seen_里面的值全部设为0
  memset(seen_, false, size_x * size_y * sizeof(bool));

  // We need to include in the inflation cells outside the bounding
  // 我们需要将边界之外的单元格包括进来
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  // 由于（min_i，min_j，max_i，max_j）是由上一层的地图已经在
  // updateBounds中更新过（这一层的updatebounds什么也没干），而InflationLayer层并未去改变它。
  // 因此这里的操作是将传入的boudns，按照机器人的膨胀尺寸，扩张这个bounds
  
  // 地图外距离达到这个距离的单元仍然会影响地图内单元储存的成本。
  
  // updatebounds膨胀的是基于米的边界，而updatecosts这里膨胀的是基于cell的边界
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);

  // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost
  //通胀列表;我们将单元格追加到一个与它到最近障碍物的距离相关的列表中
  //我们使用地图<distance, list>去模拟以前使用的优先级队列，具有显著的性能提升
  
  
  // Start with lethal obstacles: by definition distance is 0.0
  //inflation_cells_的定义如下：
  std::vector<CellData>& obs_bin = inflation_cells_[0.0];
  //接下来遍历bound中的cell，找到cost为LETHAL_OBSTACLE，即障碍物cell，
  //将其以CellData形式放进inflation_cells_[0.0]中，
  
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE)
      {
        obs_bin.push_back(CellData(index, i, j, i, j));
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  //通过增加距离来处理单元;新单元格被追加到相应的距离库中，
  //这样它们就可以超越之前插入的但距离更远的单元格
  std::map<double, std::vector<CellData> >::iterator bin;
  //外层遍历的是键，即距离障碍物的值
  //inflation_cells_.begin就是key，就是这里面对应的double
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
  {
    //内层遍历“值”，即cell。就是bin里面的std::vector<CellData>
    //他对应的就是value，就是这个距离层有多少个cell
    for (int i = 0; i < bin->second.size(); ++i)
    {
      // process all cells at distance dist_bin.first
      const CellData& cell = bin->second[i];

      unsigned int index = cell.index_;

      // ignore if already visited
      ////如果已经访问过，跳过
      if (seen_[index])
      {
        continue;
      }

      seen_[index] = true;
      //得到该cell的坐标和离它最近的障碍物的坐标
      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      // costLookup函数，它会用到我们的参考矩阵cached_costs_，
      // 通过该cell与最近障碍物的距离来确定该cell的cost，这里称为新cost
      unsigned char cost = costLookup(mx, my, sx, sy);
      //旧celcost，是master地图上的cost
      unsigned char old_cost = master_array[index];
      //inflate_unknown_为true：当新cost>0，设为新cost；
      //inflate_unknown_为false：新cost≥INSCRIBED_INFLATED_OBSTACLE，设为新cost
      /* 区别就在于，如果inflate unknown，则当膨胀到主地图上的未知区域，
        只要有cost，就覆盖它；而当inflate unknown关闭，
        当膨胀到主地图上的未知区域时，
        只有新cost是障碍物，才覆盖它，否则维持未知。后者膨胀的范围更窄！ */
      //这个就是更新index下的cost
      if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        master_array[index] = cost;
      else
      // 否则，取二者中大的cost
        master_array[index] = std::max(old_cost, cost);

      // attempt to put the neighbors of the current cell onto the inflation list
      //更新这个cell周围的cell，就是上下左右，把他们放进到inflation_cells_
      //由于下面关于enqueue的调用，最后两个参数（sx, sy)没有改变，
      //sx表示【在代价图上距离最近的障碍单元的x坐标】
      //所以保证了这个索引一定是obstacle cell。
      //由于在 enqueue入口会检查 if (!seen_[index])，
      //这保证了这些cell不会被重复的塞进去。
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy);
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy);
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy);
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy);
    }
    //至此完成一个轮次的循环，接下来不断循环（不更新已经seen_的cell），完成传播。
  }

  inflation_cells_.clear();
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
//根据(mx,my)到(src_x,src_y)的距离，把cell附近的点放到inflation_cells_里面
inline void InflationLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y)
{
  //检查这个cell是否被重复塞进去
  if (!seen_[index])
  {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    //我们将距离表计算出来的距离比膨胀半径规定的距离还要远，因此我们可以在下面进行检查
    //找它和最近的障碍物的距离，如果超过了阈值，则直接返回
    //distanceLookup里面要使用到cached_distances_，而这个是在computeCaches计算
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_)
      return;

    // push the cell data onto the inflation list and mark
    inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
  }
}
//调用computeCost函数，更新cached_costs_的值
void InflationLayer::computeCaches()
{
  //由于是基于膨胀参数来设置参考矩阵的，
  //所以当cell_inflation_radius_==0，直接返回。
  if (cell_inflation_radius_ == 0)
    return;

  //只执行一次
  /*   第二个if结构是用来初始化这两个参考矩阵的，
    只在第一次进入时执行，它们的大小都是
    (cell_inflation_radius_ + 2)x(cell_inflation_radius_ + 2)，
    设置cached_distances_矩阵的元素值为每个元素到(0,0)点的三角距离。 */
  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_)
  {
    deleteKernels();

    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
      {
        //hypot是求斜边的长度
        //就是根据下标计算距离
        //cached_distances_矩阵的元素值为每个元素到(0,0)点的三角距离。
        cached_distances_[i][j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
  {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
    {
      //将cached_distances_矩阵“翻译”到cached_costs_矩阵。
      //最终cached_costs_上，原点值为254，
      //原点到机器人足迹内切圆半径范围内为253，其余按比例下降（指数型，近处骤降），
      //cell_inflation_radius_（膨胀参数）以外没有值。
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
    }
  }
}

void InflationLayer::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }

  if (cached_costs_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_costs_[i])
        delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
    cached_costs_ = NULL;
  }
}

void InflationLayer::setInflationParameters(double inflation_radius, double cost_scaling_factor)
{
  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
  {
    // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }
}

}  // namespace costmap_2d
