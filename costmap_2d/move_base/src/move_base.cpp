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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),//加载了baseGlobalPlanner的类库
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), //加载了baseLocalPlanner的类库
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),//加载了recoveryBehaviour的类库
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {
    //as_指向action服务器，当执行as_->start()时调用MoveBase::executeCb函数
    //虽然这个是先创建的但是，刚开始是没有目标的，所以先进行的也是planthread
    //再planthread线程里面等待executeCb中的设置信号
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
      /*  enum RecoveryTrigger
        {
          PLANNING_R,
          CONTROLLING_R,
          OSCILLATION_R
        }; 
      */
     //recovery_trigger_ 恢复触发器
    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    //全局规划器，默认navfn/NavfnROS
    private_nh.param("base_global_planner", global_planner_name_, std::string("navfn/NavfnROS"));
    //局部规划器，默认TrajectoryPlannerROS
    private_nh.param("base_local_planner", local_planner_name_, std::string("base_local_planner/TrajectoryPlannerROS"));
    //robot_base_frame，默认base_link
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    //global_frame，默认/map坐标系
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    //set up plan triple buffer
    //初始化三个plan的“缓冲池”数组
   /*  geometry_msgs::PoseStamped
                std_msgs/Header header
                geometry_msgs/Pose pose
                 1                 geometry_msgs/Point position
                                  geometry_msgs/Quaternion orientation */
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    //最终由controller_plan_ 传给 local planner
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    //就是执行MoveBase::planThread方法。
    //这个线程就是等待runPlanner_被置为true
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //for commanding the base
    //发布话题
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    vel_stamped_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
    ////发布MoveBaseActionGoal消息到/move_base/goal话题上
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    
    //发布geometry_msgs::PoseStamped消息到/move_base_simple/goal话题上，
    //回调函数goalCB会处理在/move_base_simple/goal话题上接收到的消息
    //供nav_view和rviz等仿真工具使用

    //这个接受的就是movebase包订阅的move_base_simaple/goal的信息
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    //加载代价地图的参数（内切、外接、清理半径等），假设机器人的半径和costmap规定的一致
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);
    //就是调用costmap_2d_ros 接口，下面还有一个controller_costmap_ros_也是，见https://blog.csdn.net/feidaji/article/details/103178619
    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    //为计划者的成本图创建ros包装…初始化器是用于底层映射的指针
    //全局规划器代价地图global_costmap_ros_


    //Costmap2DROS类函数start()会调用各层地图的active()函数，
    //开始订阅传感器话题，对地图进行更新，
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    ////初始化全局规划器，planner_指针
    try {
      //这个就是指向特定的全局路径规划方法，比如navfn/NavfnROS
      planner_ = bgp_loader_.createInstance(global_planner_name_);
      planner_->initialize(bgp_loader_.getName(global_planner_name_), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner_name_.c_str(), ex.what());
      exit(1);
    }
    //调用costmap_2d_ros 接口 见https://blog.csdn.net/feidaji/article/details/103178619
    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    //局部规划器代价地图> 


    //Costmap2DROS类函数start()会调用各层地图的active()函数，
    //开始订阅传感器话题，对地图进行更新，
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
      //这个就是指向特定的局部路径规划方法，比如DWM
      tc_ = blp_loader_.createInstance(local_planner_name_);
      ROS_INFO("Created local_planner %s", local_planner_name_.c_str());
      tc_->initialize(blp_loader_.getName(local_planner_name_), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner_name_.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    //开启costmap基于传感器数据的更新：
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    //没见有订阅过这个服务的。所以不管他
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //advertise a service for switching global planner
    switch_global_planner_srv_ = private_nh.advertiseService("switch_global_planner", &MoveBase::switchGlobalPlannerService, this);

    //advertise a service for switching local planner
    switch_local_planner_srv_= private_nh.advertiseService("switch_local_planner", &MoveBase::switchLocalPlannerService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    //shutdown_costmaps_就是是否关闭maps
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    //开启参数动态配置服务
    as_->start();

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  bool MoveBase::switchGlobalPlannerService(any_msgs::SetString::Request& req, any_msgs::SetString::Response& res) {
    res.success = switchGlobalPlanner(req.data);
    return true;
  }

  bool MoveBase::switchGlobalPlanner(const std::string& desired_global_planner_name) {

    if(global_planner_name_ == desired_global_planner_name) {
      ROS_INFO("Global planner %s is already running!", global_planner_name_.c_str());
      return true;
    }

    boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
    //initialize the global planner
    ROS_INFO("Loading global planner %s", desired_global_planner_name.c_str());
    try {
      planner_ = bgp_loader_.createInstance(desired_global_planner_name);

      // wait for the current planner to finish planning
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

      // Clean up before initializing the new planner
      planner_plan_->clear();
      latest_plan_->clear();
      controller_plan_->clear();
      resetState();
      planner_->initialize(bgp_loader_.getName(desired_global_planner_name), planner_costmap_ros_);
      global_planner_name_ = desired_global_planner_name;
      lock.unlock();
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                 containing library is built? Exception: %s", desired_global_planner_name.c_str(), ex.what());
      planner_ = old_planner;
      return false;
    }

    return true;
  }

  bool MoveBase::switchLocalPlannerService(any_msgs::SetString::Request& req, any_msgs::SetString::Response& res) {
    res.success = switchLocalPlanner(req.data);
    return true;
  }

  bool MoveBase::switchLocalPlanner(const std::string& desired_local_planner_name) {

    if(local_planner_name_ == desired_local_planner_name) {
      ROS_INFO("Local planner %s is already running!", local_planner_name_.c_str());
      return true;
    }

    boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
    //create a local planner
    try {
      tc_ = blp_loader_.createInstance(desired_local_planner_name);
      // Clean up before initializing the new planner
      planner_plan_->clear();
      latest_plan_->clear();
      controller_plan_->clear();
      resetState();
      tc_->initialize(blp_loader_.getName(desired_local_planner_name), &tf_, controller_costmap_ros_);
      local_planner_name_ = desired_local_planner_name;
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                 containing library is built? Exception: %s", desired_local_planner_name.c_str(), ex.what());
      tc_ = old_planner;
      return false;
    }

    return true;
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;

    last_config_ = config;
  }
  //就是发布goal话题
  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::PoseStamped global_pose;

    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }


  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id.empty())
    {
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req.start;
    }

    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }
    //全局规划 MoveBase::makePlan | 调用全局规划器类方法，得到全局规划路线
    //这里面的plan就是planner_plan_
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    //如果没有全局代价地图，返回false，因为全局规划必须基于全局代价地图
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    //如果得不到机器人的起始位姿，返回false
    geometry_msgs::PoseStamped global_pose;
    //这个函数的作用就是将机器人当前位置给global_pose
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }
    //把当前位给start，作为开始位置
    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    //boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
    geometry_msgs::TwistStamped cmd_vel_stamped;
    cmd_vel_stamped.header.stamp = ros::Time::now();
    cmd_vel_stamped.header.frame_id = "base";
    cmd_vel_stamped.twist = cmd_vel;
    vel_stamped_pub_.publish(cmd_vel_stamped);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

/*
    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }
*/

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }
    //全局规划线程 void MoveBase::planThread | 调用全局规划
    //自己认为，这个线程会再executiveCB前面执行
  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      //刚开始的时候runPlanner_是false，所以会进入wait继续等待，
      //并且，将unlock planner_mutex_，这个时候executiveCB就会执行
      //executiveCB会将runPlanner_赋值为true，所以继续向下执行。
      //并且，planner_mutex_被再一次上锁
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      //将executeCb里面所执行的planner_goal_ = goal;赋值给temp_goal
      //以后全局路径规划中的目标函数就是temp_goal
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      //为啥要上面的代码要上锁呢？需要上锁保证线程安全
      //下面就是开始全局路径规划了
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      planner_plan_->clear();
      //MoveBase::makePlan作用是获取机器人的位姿作为起点，
      //然后调用全局规划器的makePlan返回规划路径，存储在planner_plan_
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan){
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        //需要对全局的变量进行更改，所以还是上锁
        //planner_plan_存储着次新
        planner_plan_ = latest_plan_;
        //latest_plan_存储着最新
        latest_plan_ = temp_plan;
        //最近一次有效全局规划的时间设为当前时间
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        //new_global_plan_ = true，表示得到了新的全局规划路线
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
        //如果全局路径规划做完了，就开始做局部路径，并且把state_标志位设置为CONTROLLING
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      //如果全局规划失败并且MoveBase还在planning状态，即机器人没有移动，则进入自转模式
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        //最迟制定出全局规划的时间=上次成功规划的时间+容忍时间
        //就是如果下次做出全局路径规划的时候大于这个最迟制定全局路径规划时间
        //说明已经超时了
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        //进入这个if语句，说明全局路径规划还没有成功
        //所以这里应该加一
        planning_retries_++;


        //还是在进行全局路径规划，并且，
        //检查时间和次数是否超过限制，若其中一项不满足限制，停止全局规划
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }
   //收到目标，触发全局规划线程，循环执行局部规划
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }
     //将目标位置转换到global坐标系下（geometry_msgs形式）
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    //we have a goal so start the planner
    //加入互斥锁，保证目标一定被传入
    //等待planthread里面的wait指令
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    //用接收到的目标goal来更新全局变量，即全局规划目标，
    //这个值在planThread中会被用来做全局规划的当前目标
    //这个时候planner_goal_的值被赋值，那么planThread也就可以执行了
    //所以下一步就是执行runPlanner_ = true这个指令
    planner_goal_ = goal;
    runPlanner_ = true;
    //开始全局规划并于此处阻塞
    //跳到planthread指令的wait指令，进行执行
    //wait指令将返回，并且再一次lock planner_mutex_
    //他会让planthread执行完或者再一次遇到wait
    planner_cond_.notify_one();
    //上面这段就是临界区
    lock.unlock();
    //全局规划完成后，发布目标到current_goal话题上
    //这么说planthread必须执行完成
    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(controller_frequency_);
    //如果代价地图是被关闭的，这里重启
    //这个代码没啥用，不用看
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    //上一次有效的局部规划时间设为现在
    last_valid_control_ = ros::Time::now();
    //上一次有效的全局规划时间设为现在
    last_valid_plan_ = ros::Time::now();
    //上一次震荡重置时间设为现在
    last_oscillation_reset_ = ros::Time::now();
    //对同一目标的全局规划次数记录归为0
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {
      //c_freq_change_被初始化为false
      //如果c_freq_change_即局部规划频率需要中途更改为真，
      //用更改后的controller_frequency_来更新r值
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }
      //如果action的服务器被抢占，
      //可能是“局部规划进行过程中收到新的目标”，
      //也可能是“收到取消行动的命令”。
      if(as_->isPreemptRequested()){
        //局部规划进行过程中收到新的目标
        if(as_->isNewGoalAvailable()){
          //这个就是在一次判断是否有新的目标进去
          //如果有就先执行完全局路径规划，再回来执行
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
          //将目标位置转换到global坐标系下（geometry_msgs形式）
          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        //说明是收到了取消行动的命令
        else {
          //如果我们被明确地抢占了先机，我们需要关闭系统
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          //通知ActionServer我们已经成功抢占
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          //action服务器清除相关内容，并调用setPreempted()函数
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      //检查目标是否被转换到全局坐标系（/map）下
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);
        //我们希望回到下一个执行周期的计划状态
        //we want to go back to the planning state for the next execution cycle
        //恢复行为索引重置为0，MoveBase状态置为全局规划中

        //为什么这里要多一个这？

        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      //在模拟中也能实时计时
      //在模拟时，如果想要进入实际运行 wall-clock time ，
      //可以用 ros::WallTime, ros::WallDuration, 和ros::WallRate，
      //类似于 ros::Time, ros::Duration, 和 ros::Rate
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      //真正进入了局部路径规划
      //调用executeCycle函数进行局部规划，传入目标和全局规划路线


      //全局路径规划做好就是一直执行在下面代码

      //这里这个global_plan没有发现用过，也不知道真正作用是啥
      //而MoveBase::planService这个服务会使用global_plan
      //但是再executeCycle里面使用的是controller_plan_
      //它是由latest_plan_这个全局变量赋值的
      //再executeCycle这里面也没有使用global_plan
      bool done = executeCycle(goal, global_plan);
      /*
          进行下面的判断
          成功结束函数
          失败判断时间
      */
      //if we're done, then we'll return from execute
      //如果局部路径规划已经做完
      //意思就是已经到达目的地，那么就会直接退出executiveCB函数
      if(done)
        return;

      //check if execution of the goal has completed in some way
      //记录从局部规划开始到这时的时间差
      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ////打印用了多长时间完成操作
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());
      //确保在我们的周期剩下的时间里睡觉
      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    //唤醒全局规划线程，以使它能够“干净地退出”
    //就是再执行一边，感觉没啥作用
    //这个节点都结束了，执行他还有啥用
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();
    //执行到这个地方，说明这个节点直接被结束了，所以Action服务器也应该关闭并返回
    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }
    //局部规划 MoveBase::executeCycle | 传入全局路线，调用局部规划器类方法，得到速度控制指令
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    //不知道这个锁是干什么的
    //会锁住函数MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level)
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    //这个就是需要发布的cmd_vel
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    
    geometry_msgs::PoseStamped global_pose;
    //获取当前的位置
    getRobotPose(global_pose, planner_costmap_ros_);
    //换成局部变量
    const geometry_msgs::PoseStamped& current_position = global_pose;

    //push the feedback out
    //feedback指的是从服务端周期反馈回客户端的信息，
    //把当前位姿反馈给客户端
    //这个就是action里面中间消息的反馈
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    //检查是否在震荡，这个是已经不是震荡了，所以进行以下的操作
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index
      //如果我们上一次的恢复是由振荡引起的，我们想要重置恢复指数
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    //检查局部规划的地图是否是当前的，否则发布零速，停止规划，制停机器人。
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    //new_global_plan_这个标志位是在执行完全全局径规划的时候被改为true
    //会在这里进行setPlan函数
    if(new_global_plan_){
      //make sure to set the new plan flag to false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;
      //把最新的路径规划点给controller_plan_这个局部路径规划
      //在指针的保护下，交换latest_plan和controller_plan的值
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      //将上一次的局部路径规划的点给latest_plan_
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");
      //boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      //在实例tc_上调用局部规划器BaseLocalPlanner的类函数setPlan()， 
      //把全局规划的结果传递给局部规划器，（这个就是这个函数的作用）
      //如果传递失败，退出并返回。
      //错误的时候才进去
      if(!tc_->setPlan(*controller_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        //这里也将全局路径规划设置为wait状态
        lock.lock();
        runPlanner_ = false;
        lock.unlock();
        //停止Action服务器，打印“将全局规划传递至局部规划器控制失败”
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      //如果我们找到有效的局部规划路线，且恢复行为是“全局规划失败”，
      //重置恢复行为索引
      //这个判断的意思就是，可能上一次全局路径规划失败了，但是这次成功了
      //所以要重置恢复行为
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //the move_base state machine, handles the control logic for navigation
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      case PLANNING:
        {
          //全局规划还没完成，还没得到一个全局路线，
          //那么唤醒一个全局规划线程去制定全局路线
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          //做完之后再回来
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //check to see if we've reached our goal
        //到达目标，停止，runPlanner_使得其再全局路径规划等待
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        //check for an oscillation condition
        //如果没到终点，检查机器人是否被困住，如果是，则进入恢复行为；
        //last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()
        //这个代码的作用就是将上一时刻的困住时间加上容忍的时间已经小于现在的时间
        //所以是被困住了
        //state_设置为CLEARING，进行恢复
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        { //这个被困住好长时间，所以让它暂停，然后进入恢复行为
          publishZeroVelocity();
          state_ = CLEARING;
          //恢复行为触发器置为，长时间困在一片小区域
          recovery_trigger_ = OSCILLATION_R;
        }

        
          //然后锁住local costmap
          //为什么锁住这个？
          //就是进行局部路径规划
        { boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
          //局部规划器实例tc_被传入了全局规划后，
          //调用computeVelocityCommands函数计算速度存储在cmd_vel中
          //如果局部路径规划成功就进行下面的代码
        if(tc_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          //若成功计算速度，上一次有效局部控制的时间设为当前
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          //这里就是发送速度指令
          vel_pub_.publish(cmd_vel);
          geometry_msgs::TwistStamped cmd_vel_stamped;
          cmd_vel_stamped.header.stamp = ros::Time::now();
          cmd_vel_stamped.header.frame_id = "base"; // TODO: make it configurable
          cmd_vel_stamped.twist = cmd_vel;
          vel_stamped_pub_.publish(cmd_vel_stamped);
          //如果之前是局部路径规划出现错误，
          //那么这次就是将上一次的标志位进行清零
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else {
          //局部路径规划失败
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
          //check if we've tried to find a valid control for longer than our time limit
          //ros::Time::now() > attempt_end这个就是说明已经超时了
          //超时的话就进入恢复行为
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else{
            //说明没有超时
            //发布0速度，在机器人当前位置再次回到全局规划
            //otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            //再进行全局路径规划的时候会将state_更改为CONTROLLING
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            //进入全局路径规划,
            //再进行全局路径规划的时候会将state_更改为CONTROLLING.
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      //如果全局规划失败，进入了恢复行为状态，
      //我们尝试去用用户提供的恢复行为去清除空间
      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //如果它们被启用（recovery_behavior_enabled_），我们将调用我们当前所使用的任何恢复行为
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        //如果允许使用恢复行为，且恢复行为索引值小于恢复行为数组的大小
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
          //开始恢复行为，在executeCycle的循环中一次次迭代恢复行为
          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          //上一次震荡重置时间设为现在
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        else{
          //若恢复行为无效，则进行下面的操作
          //就是里面的恢复行为都实验完了，还是不行，那么就进行到这里面

          //打印“所有的恢复行为都失败了，关闭全局规划器”
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");
          //反馈失败的具体信息
          //根据上面的信息进行反馈
          //具有三种情况，可以进入到恢复行为
          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
