/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
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
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.h 3293 2005-11-19 08:37:45Z gerkey $
 *************************************************************************/

#ifndef PF_H
#define PF_H

#include "pf_vector.h"
#include "pf_kdtree.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
struct _pf_t;
struct _rtk_fig_t;
struct _pf_sample_set_t;

// Function prototype for the initialization model; generates a sample pose from
// an appropriate distribution.
//初始化模型的函数原型;根据适当的分布生成一个样本位姿。
typedef pf_vector_t (*pf_init_model_fn_t) (void *init_data);

// Function prototype for the action model; generates a sample pose from
// an appropriate distribution
typedef void (*pf_action_model_fn_t) (void *action_data, 
                                      struct _pf_sample_set_t* set);

// Function prototype for the sensor model; determines the probability
// for the given set of sample poses.
typedef double (*pf_sensor_model_fn_t) (void *sensor_data, 
                                        struct _pf_sample_set_t* set);


// Information for a single sample
//一个简单的粒子
//而对于每个粒子，
//我们都有字段pose描述位姿， weight用于描述粒子的概率权重。
typedef struct
{
  // Pose represented by this sample
  pf_vector_t pose;

  // Weight for this pose
  double weight;
  
} pf_sample_t;


// Information for a cluster of samples
//粒子族的数据结构
//虽然粒子簇也是粒子的集合，
//它只是描述了簇的粒子数量、概率权重、均值方差等统计信息。
//就是子峰的权重位置和协方差
typedef struct
{
  // Number of samples
  int count;

  // Total weight of samples in this cluster
  double weight;

  // Cluster statistics
  pf_vector_t mean;
  pf_matrix_t cov;

  // Workspace
  double m[4], c[2][2];
  
} pf_cluster_t;


// Information for a set of samples
//样本集合
typedef struct _pf_sample_set_t
{
  // The samples
  //记录的是样本的数量
  int sample_count;
  //样本，是一个一维数组
  pf_sample_t *samples;

  // A kdtree encoding the histogram
  //kdtree则用于描述样本的统计直方图
  pf_kdtree_t *kdtree;

  // Clusters
  //针对多峰的分布，字段clusters来显式的描述，
  //我们把描述各个峰的子集称为粒子簇。
  // 此外cluster_count和cluster_max_count分别描述了
  //当前粒子簇的数量和最大粒子簇数量，就是有多少峰值
  int cluster_count, cluster_max_count;
  pf_cluster_t *clusters;

  // Filter statistics
  //字段mean和cov分别描述了当前粒子集合的平均位姿和协方差，
  //它们是对当前机器人位姿和不确定度的估计。
  pf_vector_t mean;
  pf_matrix_t cov;
  int converged; 
} pf_sample_set_t;


// Information for an entire filter
//一个滤波器
typedef struct _pf_t
{
  // This min and max number of samples
  // 最少和最多样本数量
  int min_samples, max_samples;

  // Population size parameters
  // 样本集尺寸参数
  //pop_err真实分布和估计分布之间的最大误差。
  double pop_err, pop_z;
  
  // The sample sets.  We keep two sets and use [current_set]
  // to identify the active set.
  //amcl定义了两个样本集合，并使用current_set指示当前激活的集合索引。
  // 当前激活态的样本集索引
  int current_set;
  // 样本集合,双缓存
  pf_sample_set_t sets[2];

  // Running averages, slow and fast, of likelihood
  double w_slow, w_fast;

  // Decay rates for running averages
  double alpha_slow, alpha_fast;

  // Function used to draw random pose samples
  // 绘制样本集合的函数指针
  pf_init_model_fn_t random_pose_fn;
  void *random_pose_data;
  // 判定粒子集合发散的阈值
  double dist_threshold; //distance threshold in each axis over which the pf is considered to not be converged
  // 粒子集合收敛的标志
  int converged; 
} pf_t;


// Create a new filter
pf_t *pf_alloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               pf_init_model_fn_t random_pose_fn, void *random_pose_data);

// Free an existing filter
void pf_free(pf_t *pf);

// Initialize the filter using a guassian
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov);

// Initialize the filter using some model
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data);

// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn, void *action_data);

// Update the filter with some new sensor observation
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn, void *sensor_data);

// Resample the distribution
void pf_update_resample(pf_t *pf);

// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var);

// Compute the statistics for a particular cluster.  Returns 0 if
// there is no such cluster.
int pf_get_cluster_stats(pf_t *pf, int cluster, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov);

// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set);


// Display the sample set
void pf_draw_samples(pf_t *pf, struct _rtk_fig_t *fig, int max_samples);

// Draw the histogram (kdtree)
void pf_draw_hist(pf_t *pf, struct _rtk_fig_t *fig);

// Draw the CEP statistics
void pf_draw_cep_stats(pf_t *pf, struct _rtk_fig_t *fig);

// Draw the cluster statistics
void pf_draw_cluster_stats(pf_t *pf, struct _rtk_fig_t *fig);

//calculate if the particle filter has converged - 
//and sets the converged flag in the current set and the pf 
int pf_update_converged(pf_t *pf);

//sets the current set and pf converged values to zero
void pf_init_converged(pf_t *pf);

#ifdef __cplusplus
}
#endif


#endif
