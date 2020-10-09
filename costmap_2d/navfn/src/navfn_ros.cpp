/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#include <navfn/navfn_ros.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(navfn::NavfnROS, nav_core::BaseGlobalPlanner)

namespace navfn {

  NavfnROS::NavfnROS() 
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {}

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap, global_frame);
  }
  //这个函数是在movebase里面被调用
  //planner_->initialize(bgp_loader_.getName(global_planner_name_), planner_costmap_ros_);
  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame){
    //
    if(!initialized_){
      //这个就是全局代价函数
      costmap_ = costmap;
      //全局id来自全局代价函数的id
      global_frame_ = global_frame;
      //指向NavFn类实例，传入参数为地图大小
      planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));

      ros::NodeHandle private_nh("~/" + name);
      //ros::Publisher plan_pub_;
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      private_nh.param("visualize_potential", visualize_potential_, false);

      //if we're going to visualize the potential array we need to advertise
       //如果要将potential array可视化，
       //则发布节点名称下的/potential话题，需要的用户可以订阅
      if(visualize_potential_)
        // 发布这个话题
        potarr_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("potential", 1);

      private_nh.param("allow_unknown", allow_unknown_, true);
      private_nh.param("planner_window_x", planner_window_x_, 0.0);
      private_nh.param("planner_window_y", planner_window_y_, 0.0);
      private_nh.param("default_tolerance", default_tolerance_, 0.0);
      //发布make_plan的服务
      //没见过有订阅这个服务的。所以不管他
      make_plan_srv_ =  private_nh.advertiseService("make_plan", &NavfnROS::makePlanService, this);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point){
    return validPointPotential(world_point, default_tolerance_);
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point, double tolerance){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    double resolution = costmap_->getResolution();
    geometry_msgs::Point p;
    p = world_point;

    p.y = world_point.y - tolerance;

    while(p.y <= world_point.y + tolerance){
      p.x = world_point.x - tolerance;
      while(p.x <= world_point.x + tolerance){
        double potential = getPointPotential(p);
        if(potential < POT_HIGH){
          return true;
        }
        p.x += resolution;
      }
      p.y += resolution;
    }

    return false;
  }


  //获得世界上给定点上的潜在或导航成本(注意:您应该首先调用computePotential)
  double NavfnROS::getPointPotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return -1.0;
    }

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return DBL_MAX;

    unsigned int index = my * planner_->nx + mx;
    //potarr数组记录的对应cell的Potential值
    //就是potarr数组存储着各个cell的potential值
    return planner_->potarr[index];
  }

  bool NavfnROS::computePotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return false;

    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    return planner_->calcNavFnDijkstra();
  }

  void NavfnROS::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  bool NavfnROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp){
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
  } 

  void NavfnROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    return makePlan(start, goal, default_tolerance_, plan);
  }
  //if(!planner_->makePlan(start, goal, plan) || plan.empty())
  //被上面的函数调用
  //注意上面的planner_不是这里的planner_，
  //上面的boost::shared_ptr<nav_core::BaseGlobalPlanner>
  //下面的是boost::shared_ptr<NavFn> planner_;
  //其实他们是一样的，movebase调用自己的planner_的makeplan，
  //但是这个使得纯虚函数，所以会调用NavFn重写的makeplan函数
  //这里面的plan就是movebase里面的planner_plan_全局参数
  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::mutex::scoped_lock lock(mutex_);
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    if(start.header.frame_id != global_frame_){
      ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), start.header.frame_id.c_str());
      return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    //因为map地图上的是按照一个单元一个单元进行定义的 
    //而world这是通过实际的meter进行定义的，所以要进行转换
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    //清理起始位置cell（必不是障碍物）
    clearRobotCell(start, mx, my);

    //make sure to resize the underlying array that Navfn uses
    //确保调整Navfn使用的底层数组的大小
    //setNavArr这个函数创建四个数组（除了创建没有干别的事情）
    //costarr数组（记录全局costmap信息）、
    //potarr数组（储存各cell的Potential值）就是代价值表
    //以及x和y向的梯度数组（用于生成路径）
    //还有一个pending数组
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    //costmap_->getCharMap()
    //将返回一个指针，指向作为costmap使用的底层无符号字符数组。
    //调用setCostmap函数给全局costmap赋值。
    //setCostmap将Costmap“翻译”成costarr
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if(!costmap_->worldToMap(wx, wy, mx, my)){
      if(tolerance <= 0.0){
        ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
        return false;
      }
      mx = 0;
      my = 0;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;
    //就是从目标往起点进行行走
    planner_->setStart(map_goal);
    planner_->setGoal(map_start);

    //bool success = planner_->calcNavFnAstar();
    //调用该类的calcNavFnDijkstra函数，这个函数可以完成全局路径的计算。
    //就是调用D*算法进行求解全局路径规划
    planner_->calcNavFnDijkstra(true);
    //得到地图的分辨率
    double resolution = costmap_->getResolution();
    //因为这个map是按照分辨率进行切分的cell
    //所以说不可能goal正好是在cell上，
    //所以要做的就是找到最接近goal的cell
    geometry_msgs::PoseStamped p, best_pose;
    p = goal;
    //将标志位--找到合法的best_pose置零
    bool found_legal = false;
    //与DBL_MAX比较，
    double best_sdist = DBL_MAX;

    p.pose.position.y = goal.pose.position.y - tolerance;
    //就是从左下角cell开始找，直到找到所需的值
    //就是每一个cell都遍历一遍
    while(p.pose.position.y <= goal.pose.position.y + tolerance){
      p.pose.position.x = goal.pose.position.x - tolerance;
      while(p.pose.position.x <= goal.pose.position.x + tolerance){
        double potential = getPointPotential(p.pose.position);
        //距离是平方值
        double sdist = sq_distance(p, goal);
        if(potential < POT_HIGH && sdist < best_sdist){
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        //这个就是相加一个cell
        p.pose.position.x += resolution;
      }
      p.pose.position.y += resolution;
    }

    if(found_legal){
      //extract the plan
      //获取规划结果
          //因为这个map是按照分辨率进行切分的cell
          //所以说不可能goal正好是在cell上，
          //所以要做的就是找到最接近goal的cell
      //这个函数就是将新的goal与起点重新生成一个路径
      if(getPlanFromPotential(best_pose, plan)){
        //make sure the goal we push on has the same timestamp as the rest of the plan
        //确保我们推进的目标与计划的其他部分具有相同的时间戳
        //里面的路径已经有这个best_pose，为什么还要加呢
        //我觉得这个goal_copy应该是用goal进行赋值的
        geometry_msgs::PoseStamped goal_copy = best_pose;
        goal_copy.header.stamp = ros::Time::now();
        plan.push_back(goal_copy);
      }
      else{
        ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
      }
    }
    //这个函数就是根据点云数据，就每个cell的Potential值赋给他们并且发布出去
    if (visualize_potential_)
    {
      // Publish the potentials as a PointCloud2
      sensor_msgs::PointCloud2 cloud;
      cloud.width = 0;
      cloud.height = 0;
      cloud.header.stamp = ros::Time::now();
      cloud.header.frame_id = global_frame_;
      sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
      cloud_mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                        "y", 1, sensor_msgs::PointField::FLOAT32,
                                        "z", 1, sensor_msgs::PointField::FLOAT32,
                                        "pot", 1, sensor_msgs::PointField::FLOAT32);
      cloud_mod.resize(planner_->ny * planner_->nx);
      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");

      PotarrPoint pt;
      //potarr数组（储存各cell的Potential值）
      float *pp = planner_->potarr;
      double pot_x, pot_y;
      for (unsigned int i = 0; i < (unsigned int)planner_->ny*planner_->nx ; i++)
      {
        if (pp[i] < 10e7)
        {
          mapToWorld(i%planner_->nx, i/planner_->nx, pot_x, pot_y);
          iter_x[0] = pot_x;
          iter_x[1] = pot_y;
          iter_x[2] = pp[i]/pp[planner_->start[1]*planner_->nx + planner_->start[0]]*20;
          iter_x[3] = pp[i];
          ++iter_x;
        }
      }
      potarr_pub_.publish(cloud);
    }

    //publish the plan for visualization purposes
    //发布计划以实现可视化，跟上面的实现点云的potential不一样
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    //如果都成功，就是返回true
    return !plan.empty();
  }
  //就是发布规划的路径
  void NavfnROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(!path.empty())
    {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }
  // 它在makePlan中被调用，
  // 主要工作是调用了NavFn类的一些函数，设置目标、获取规划结果。
  // getPlanFromPotential(best_pose, plan)只有这一个函数调用这个函数
  bool NavfnROS::getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }
    //储存bestpose的坐标
    double wx = goal.pose.position.x;
    double wy = goal.pose.position.y;

    //the potential has already been computed, so we won't update our copy of the costmap
    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;
    ////调用navfn的设置起始、calcPath、getPathX等函数，
    //并将计算出的路径点依次存放plan，得到全局规划路线


    //以目标作为开始点
    //map_goal就是传过来的goal的map中的距离map中最近的的bestpose
    planner_->setStart(map_goal);
    //该函数负责在potarr数组的基础上选取一些cell点来生成最终的全局规划路径，
    //从起点开始沿着最优行走代价值梯度下降的方向寻找到目标点的最优轨迹。
    planner_->calcPath(costmap_->getSizeInCellsX() * 4);

    //extract the plan
    float *x = planner_->getPathX();
    float *y = planner_->getPathY();
    int len = planner_->getPathLen();
    ros::Time plan_time = ros::Time::now();
    //这个函数的作用就是将xy中的值放到plan中
    //注意，这里是反着放的，因为上面是将目标作为起始值，
    //那么现在反过来，也就是起点就是起始值了
    for(int i = len - 1; i >= 0; --i){
      //convert the plan to world coordinates
      double world_x, world_y;
      mapToWorld(x[i], y[i], world_x, world_y);

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return !plan.empty();
  }
};
