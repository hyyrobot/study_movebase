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
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "amcl/pf/pf.h"
#include "amcl/pf/pf_pdf.h"
#include "amcl/pf/pf_kdtree.h"


// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t *pf, int k);



// Create a new filter
//创建一个新的滤波器
//该函数有6个输入参数，前四个是对滤波器的配置，约束了最少和最多的样本数量，
//权重过滤值衰减速率。 
//参数random_pose_fn是一个函数指针，提供了生成随机样本的函数，
//amcl中使用的是AmclNode::uniformPoseGenerator。
//对于参数random_pose_data，是粒子的样本空间， amcl实际使用的是地图对象。
pf_t *pf_alloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               pf_init_model_fn_t random_pose_fn, void *random_pose_data)
{
  int i, j;
  pf_t *pf;//一个滤波器
  pf_sample_set_t *set;//一个样本集合
  pf_sample_t *sample;//一个样本
  
  srand48(time(NULL));//伪随机数产生器

  pf = calloc(1, sizeof(pf_t));//生成一个滤波器

  pf->random_pose_fn = random_pose_fn;
  pf->random_pose_data = random_pose_data;

  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  // 用于总体大小计算的控制参数。 [err]是真实分布与估计值之间的最大误差
  // 分配。 [z]是（1-p），其中p是估计分布偏差的误差小于[err]的概率。
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5; //判定粒子集合发散的阈值
  
  pf->current_set = 0;

  // 接着通过一个for循环完成两个粒子集合的初始化工作。
  // 并在7到13行中的为每个粒子集申请内存，并赋予初值。
  // 在第14行中构建了三倍于样本集合尺寸的直方图对象kdtree。 
  // 最后完成粒子簇的内存申请和均值方差的初始化。

  //因为滤波使用的是双集合，双缓存，所以这里的sets应该是2
  //也就是有两个样本集pf_sample_set_t
  for (j = 0; j < 2; j++)
  {
    set = pf->sets + j;
      
    set->sample_count = max_samples;
    set->samples = calloc(max_samples, sizeof(pf_sample_t));
    //初始值都是0（x y theta）权重就是1/n
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
    }

    // HACK: is 3 times max_samples enough?
    //构建了三倍于样本集合尺寸的直方图对象kdtree
    //已经经过证明最大是2*n-1，所以说三倍是足够的
    set->kdtree = pf_kdtree_alloc(3 * max_samples);
    //针对多峰的分布，字段clusters来显式的描述，
    //cluster_count就是峰值的数量
    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));
    //期望值和协方差都是0
    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;

  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  //set converged to 0
  //设置收敛于为0
  pf_init_converged(pf);

  return pf;
}

// Free an existing filter
void pf_free(pf_t *pf)
{
  int i;
  
  for (i = 0; i < 2; i++)
  {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);
  
  return;
}

// Initialize the filter using a guassian
//该函数有三个参数，其中pf是将要初始化的滤波器对象，
//mean和cov分别是机器人初始位姿和协方差描述。
//根据这些参数为粒子滤波器生成sample_count个粒子
//并且将里面的粒子进行聚类，然后计算这个聚类（也就是单峰）的均值和协方差
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  pf_pdf_gaussian_t *pdf;//并根据输入的均值和方差构建一个高斯分布。
  //首先通过索引current_set获取当前激活的粒子集合。
  set = pf->sets + pf->current_set;
  
  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;
  //创建一个高斯概率密度函数，就是高斯分布
  //有期望、协方差、还有分解后的矩阵、再产生一个随机值
  //创建这个是干啥的呢？
  //就是下面产生粒子的，粒子就是为粒子滤波器pf做准备的
  //这些粒子就是sample中的pose
  pdf = pf_pdf_gaussian_alloc(mean, cov);
    
  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    //就是随机产生一个位姿参数
    sample->pose = pf_pdf_gaussian_sample(pdf);

    // Add sample to histogram
    //把这个位姿参数放到kdtree里面
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;
  //产生粒子之后就释放这个pdf
  //最后释放掉临时申请的高斯分布对象pdf，
  pf_pdf_gaussian_free(pdf);
    
  // Re-compute cluster statistics
  //这个函数就是将里面的粒子进行聚类，然后计算这个聚类（也就是单峰）的均值和协方差
  pf_cluster_stats(pf, set); 

  //set converged to 0
  //并标记为未收敛。
  pf_init_converged(pf);

  return;
}


// Initialize the filter using some model
// 它完成的功能与pf_init基本是一样的，只是pf_init使用的是高斯模型，
//pf_init_model可以由用户提供模型。 
//它有三个参数，其中pf仍然是滤波器对象。
//init_fn是一个函数指针，用户需要提供一个生成随机样本的函数实现，
//该参数的作用就相当于pf_init中的高斯分布对象。
//参数init_data则是生成粒子的样本空间， 我们可以为之赋予地图数据。
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = (*init_fn) (init_data);

    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);
  
  //set converged to 0
  pf_init_converged(pf);

  return;
}

void pf_init_converged(pf_t *pf){
  pf_sample_set_t *set;
  //current_set就是说当前使用的哪一个粒子集
  set = pf->sets + pf->current_set;
  set->converged = 0; 
  pf->converged = 0; 
}

int pf_update_converged(pf_t *pf)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;
  //就是重采样作用的集合
  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;
  //对每个粒子进行判断
  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;
    if(fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold || 
       fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold){
      set->converged = 0; 
      pf->converged = 0; 
      return 0;
    }
  }
  set->converged = 1; 
  pf->converged = 1; 
  return 1; 
}

// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn, void *action_data)
{
  pf_sample_set_t *set;

  set = pf->sets + pf->current_set;

  (*action_fn) (action_data, set);
  
  return;
}


#include <float.h>
// Update the filter with some new sensor observation
//下面是pf_update_sensor函数的代码片段，
//它完成的是短期和长期样本均值的估计， 也就是字段w_slow和w_fast的计算。

//它有三个参数，
//其中pf是滤波器对象，
//sensor_fn则是一个函数指针实现了传感器的模型，
//用于计算各个粒子的概率权重。参数sensor_data则是用于更新的传感器数据。
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn, void *sensor_data)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;
  //current_set就是说当前使用的哪一个粒子集
  set = pf->sets + pf->current_set;

  // Compute the sample weights
  //这是一个函数，是会调用观测模型函数的，就是前面说的三个
  //算法原理请参看《概率机器人》6.4节，
  //这里会给每个粒子（代表一个位姿）计算概率、更新概率，然后返回所有粒子概率的总和。
  //第一个参数就是雷达数据  第二个参数就是粒子集
  total = (*sensor_fn) (sensor_data, set);
  
  if (total > 0.0)
  {
    // Normalize weights
    //接下来首先完成样本权重的归一化。
          double w_avg=0.0;
          for (i = 0; i < set->sample_count; i++)
          {
            sample = set->samples + i;
            w_avg += sample->weight;
            //就是指针，就是归一化各个权重
            sample->weight /= total;
          }
          //更新w_avg这个参数
          w_avg /= set->sample_count;
    // Update running averages of likelihood of samples (Prob Rob p258)
    //最后按照右边算法伪代码中的第10和11行的公式，计算统计量wslow,wfast
          if(pf->w_slow == 0.0)
            pf->w_slow = w_avg;
          else
            pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);
          if(pf->w_fast == 0.0)
            pf->w_fast = w_avg;
          else
            pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    //printf("w_avg: %e slow: %e fast: %e\n", 
           //w_avg, pf->w_slow, pf->w_fast);
  }
  else
  {
    //如果total为0 权重就是1/n
    // Handle zero total
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }

  return;
}


// Resample the distribution
void pf_update_resample(pf_t *pf)
{
  int i;
  double total;
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b;

  //double r,c,U;
  //int m;
  //double count_inv;
  double* c;

  double w_diff;
  //就是两个粒子集合
  //一个用来接收当前粒子集的信息（set_a），
  //一个用来用作重采样的缓存粒子集(set_b)。
  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  c = (double*)malloc(sizeof(double)*(set_a->sample_count+1));
  //就是进行将1分成sample_count个区间，然后随机产生数字落于区间，落到哪个区间
  //就是在哪个区间生成一个粒子
  c[0] = 0.0;
  for(i=0;i<set_a->sample_count;i++)
    c[i+1] = c[i]+set_a->samples[i].weight;
  
  //我们重置更新粒子集合set_b，它将保存重采样后的粒子。
      // Create the kd tree for adaptive sampling
      pf_kdtree_clear(set_b->kdtree); 
      // Draw samples from set a to create set b.
      total = 0;
      set_b->sample_count = 0;
  

  //接着计算算法Augmented_MCL中第13行的判据，该判据是用于判定是否需要插入新粒子。
  //首先在注入粒子前需要引入两个与之有关的参数 
  //alpha_slow 和 alpha_fast其分别为估计长期和短期平均的指数滤波器的衰减率。
  //并且这两个参数满足0<alpha_slow << alpha_fast 。
  //在ROS导航包中这两个参数分别默认设为0.001和0.1（可自定义）。 
  //这两个参数对w_fast (长期似然平均）与 w_slow（短期似然平均）有直接影响。
  //当系统出现机器人绑架等问题时，粒子的平均权重会开始下降，
  //此时随着机器人的运动与粒子不断的测量更新，
  //粒子平均权重将会保持在一个低位。
  //从而导致w_slow 的值会大于w_fast,使得0<w_fast /w_slow<1。
  //此时粒子滤波器将会按照max(0.0, 1.0 –w_fast /w_slow)的概率往粒子集里面注入随机粒子。
  //当系统对自己的位置十分肯定后w_fast >w_slow ，此时随机粒子不再添加，
  //而是从原先的粒子集中筛选权重较大的。
      w_diff = 1.0 - pf->w_fast / pf->w_slow;
      if(w_diff < 0.0)
        w_diff = 0.0;
      //printf("w_diff: %9.6f\n", w_diff);

  // Can't (easily) combine low-variance sampler with KLD adaptive
  // sampling, so we'll take the more traditional route.
  /*
  // Low-variance resampler, taken from Probabilistic Robotics, p110
  count_inv = 1.0/set_a->sample_count;
  r = drand48() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  m = 0;
  */
 //就是从0开始生成粒子，直到数量和max_samples一样
  while(set_b->sample_count < pf->max_samples)
  {
    sample_b = set_b->samples + set_b->sample_count++;
    //这个的意思得好好揣摩
    //if什么时候不成立呢，就是w_diff为0，什么时候为0呢？就是w_fast<<w_slow
    //也就是没有发生绑架
    //这样就进行下面的else，就是根据权重进行重采样（就是将sample_a）导进sample_b中
    //如果运行if，就是随机的加入点
    //https://zhuanlan.zhihu.com/p/59663340 看这个博客
    //w_diff表示上文中讨论过的注入粒子的概率（1.0 –w_fast /w_slow）。
    //当产生的随机数小于w_diff时，将往set_b中随机注入粒子。
    //当w_diff<0 或者产生的随机数大于等于w_diff时都不注入粒子，
    //而是从set_a中挑选粒子放进set_b中。
    if(drand48() < w_diff)
      //pf->random_pose_fn为一个函数指针，其返回一个随机位姿
      sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
    else
    {
      // Can't (easily) combine low-variance sampler with KLD adaptive
      // sampling, so we'll take the more traditional route.
      /*
      // Low-variance resampler, taken from Probabilistic Robotics, p110
      U = r + m * count_inv;
      while(U>c)
      {
        i++;
        // Handle wrap-around by resetting counters and picking a new random
        // number
        if(i >= set_a->sample_count)
        {
          r = drand48() * count_inv;
          c = set_a->samples[0].weight;
          i = 0;
          m = 0;
          U = r + m * count_inv;
          continue;
        }
        c += set_a->samples[i].weight;
      }
      m++;
      */

      // Naive discrete event sampler
      //生成的随机数在c[i]与c[i+1]之间，那么就把这这个区间所对应的粒子放进sample_b中
      double r;
      r = drand48();
      for(i=0;i<set_a->sample_count;i++)
      {
        if((c[i] <= r) && (r < c[i+1]))
          break;
      }
      assert(i<set_a->sample_count);

      sample_a = set_a->samples + i;

      assert(sample_a->weight > 0);

      // Add sample to list
      sample_b->pose = sample_a->pose;
    }

    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    //但是还有一个问题便是粒子的数量。
    //在函数的开头我们要确保set_b的粒子数不能超过粒子集的最大数量。
    //但即使这样粒子数量还是很大（比如5000）。
    //这么大的粒子数在定位初期的确能提高定位的精度，
    //但是当粒子集开始聚合后，程序便不再需要这么多的粒子，
    //因此这里需要引入一个新的控制条件来节省计算资源。
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count))
      break;
  }
  
  // Reset averages, to avoid spiraling off into complete randomness.
  if(w_diff > 0.0)
    pf->w_slow = pf->w_fast = 0.0;

  //fprintf(stderr, "\n\n");

  // Normalize weights
  //对粒子的权重归一化，为了之后的聚类分析做准备。
  for (i = 0; i < set_b->sample_count; i++)
  {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }
  
  // Re-compute cluster statistics
  //聚类计算
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  //使用sample_b中的数据
  pf->current_set = (pf->current_set + 1) % 2; 

  pf_update_converged(pf);

  free(c);
  return;
}


// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int pf_resample_limit(pf_t *pf, int k)
{
  double a, b, c, x;
  int n;

  if (k <= 1)
    return pf->max_samples;

  a = 1;
  b = 2 / (9 * ((double) k - 1));
  c = sqrt(2 / (9 * ((double) k - 1))) * pf->pop_z;
  x = a - b + c;

  n = (int) ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

  if (n < pf->min_samples)
    return pf->min_samples;
  if (n > pf->max_samples)
    return pf->max_samples;
  
  return n;
}


// Re-compute the cluster statistics for a sample set
//重新计算样本集的集群统计信息
// 计算某一聚类的统计特性, amcl_node.cpp中根据聚类，获取权重最高的聚类的统计特性，即为当前机器人所在的位姿
// 注意set 和 cluster的区别   另外，第一个参数没用上啊，可能两个形参有关系？？？
/*
   这个函数的思路，传入 set 指针，对这个指针进行一系列操作：
   1、将set->kdtree 中的节点分群
   2、按照 cluster_max_count 初始化所有的 cluster 
   3、初始化 filter stats 和 cluster stats
   4、按照 sample_count 给每个 sample 找cluster 并且合理的增加 cluster_max_count 以及set->cluster_count
   5、通过 cluster 指针，修改 set->cluster 中的数据
   6、如果一类中有多个粒子（sample），cluster->count += 1 ， 同时权重 weight += sample->weight
   7、根据权重计算均值cluster->m[0] += sample->weight * sample->pose.v[0]，
   与此同时overall filter 的信息也得到修改，m[0] += sample->weight * sample->pose.v[0];
   8、每进来一个粒子（sample） 就计算 cluster->c[] ， 这是每一个类（种群）的协方差（不是真的协方差，还差一步）。同时修改 overall filter
   9、归一化，对每一个类（种群）进行归一化处理，sum/weight ， 同时计算协方差，协方差公式就是。。。额百度吧，打公式太费劲
   10、Compute overall filter stats 上面是计算每个群的 平均值、协方差矩阵，这里是计算多个群组成所有的粒子的 stats
*/
//这个函数就是将里面的粒子进行聚类，然后计算这个聚类（也就是单峰）的均值和协方差
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set)
{
  int i, j, k, cidx;
  pf_sample_t *sample;
  pf_cluster_t *cluster;
  
  // Workspace
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  pf_kdtree_cluster(set->kdtree);
  
  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++)
  {
    cluster = set->clusters + i;
    cluster->count = 0;
    cluster->weight = 0;
    cluster->mean = pf_vector_zero();
    cluster->cov = pf_matrix_zero();

    for (j = 0; j < 4; j++)
      cluster->m[j] = 0.0;
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->c[j][k] = 0.0;
  }

  // Initialize overall filter stats
  count = 0;
  weight = 0.0;
  set->mean = pf_vector_zero();
  set->cov = pf_matrix_zero();
  for (j = 0; j < 4; j++)
    m[j] = 0.0;
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      c[j][k] = 0.0;
  
  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    //printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2]);

    // Get the cluster label for this sample
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count)
      continue;
    if (cidx + 1 > set->cluster_count)
      set->cluster_count = cidx + 1;
    
    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
      {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
  }

  // Normalize
  for (i = 0; i < set->cluster_count; i++)
  {
    cluster = set->clusters + i;
        
    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);

    cluster->cov = pf_matrix_zero();

    // Covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
          cluster->mean.v[j] * cluster->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                                         cluster->m[3] * cluster->m[3]));

    //printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count, cluster->weight,
           //cluster->mean.v[0], cluster->mean.v[1], cluster->mean.v[2]);
    //pf_matrix_fprintf(cluster->cov, stdout, "%e");
  }

  // Compute overall filter stats
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));

  return;
}


// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var)
{
  int i;
  double mn, mx, my, mrr;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  
  set = pf->sets + pf->current_set;

  mn = 0.0;
  mx = 0.0;
  my = 0.0;
  mrr = 0.0;
  
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    mn += sample->weight;
    mx += sample->weight * sample->pose.v[0];
    my += sample->weight * sample->pose.v[1];
    mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
    mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
  }

  mean->v[0] = mx / mn;
  mean->v[1] = my / mn;
  mean->v[2] = 0.0;

  *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));

  return;
}


// Get the statistics for a particular cluster.
//得到粒子集中单峰的权重均值和协方差
int pf_get_cluster_stats(pf_t *pf, int clabel, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov)
{
  pf_sample_set_t *set;
  pf_cluster_t *cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count)
    return 0;
  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}


