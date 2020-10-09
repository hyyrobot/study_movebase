/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: Odometry sensor model for AMCL
// Author: Andrew Howard
// Date: 17 Aug 2003
// CVS: $Id: amcl_odom.h 4135 2007-08-23 19:58:48Z gerkey $
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_ODOM_H
#define AMCL_ODOM_H

#include "amcl_sensor.h"
#include "../pf/pf_pdf.h"

namespace amcl
{

typedef enum
{
  //其中diff是按照《P.R.》中的sample_motion_model_odometry算法来实现的
  //使用odom_alpha1到odom_alpha4来描述噪声
  ODOM_MODEL_DIFF,
  //mni则是在diff基础上增加了一个参数odom_alpha5来描述垂直与运动方向的平移趋势。
  ODOM_MODEL_OMNI,
  //下面两个就是为了弥补不足进行改进的
  ODOM_MODEL_DIFF_CORRECTED,
  ODOM_MODEL_OMNI_CORRECTED
} odom_model_t;

// Odometric sensor data
//继承父类AMCLSensorData
//它在基类的基础上增加了pose和delta两个字段，
//分别用于记录里程计的位姿和运动量。
//delta也是三维向量，x y theta
//它们将用作运动更新的控制量输入ut。
class AMCLOdomData : public AMCLSensorData
{
  // Odometric pose
  public: pf_vector_t pose;

  // Change in odometric pose
  public: pf_vector_t delta;
};


// Odometric sensor model
class AMCLOdom : public AMCLSensor
{
  // Default constructor
  public: AMCLOdom();

  public: void SetModelDiff(double alpha1, 
                            double alpha2, 
                            double alpha3, 
                            double alpha4);

  public: void SetModelOmni(double alpha1, 
                            double alpha2, 
                            double alpha3, 
                            double alpha4,
                            double alpha5);

  public: void SetModel( odom_model_t type,
                         double alpha1,
                         double alpha2,
                         double alpha3,
                         double alpha4,
                         double alpha5 = 0 );

  // Update the filter based on the action model.  Returns true if the filter
  // has been updated.
  //而函数UpdateAction则是根据里程计数据@data来更新粒子滤波器@pf， 如果成功更新则返回true。
  public: virtual bool UpdateAction(pf_t *pf, AMCLSensorData *data);

  // Current data timestamp
  private: double time;
  
  // Model type
  private: odom_model_t model_type;

  // Drift parameters
  private: double alpha1, alpha2, alpha3, alpha4, alpha5;
};


}

#endif
