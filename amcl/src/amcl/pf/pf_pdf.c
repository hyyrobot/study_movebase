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
 * Desc: Useful pdf functions
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf_pdf.c 6348 2008-04-17 02:53:17Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//#include <gsl/gsl_rng.h>
//#include <gsl/gsl_randist.h>

#include "amcl/pf/pf_pdf.h"

// Random number generator seed value
//随机数发生器的种子值
static unsigned int pf_pdf_seed;


/**************************************************************************
 * Gaussian
 *************************************************************************/

// Create a gaussian pdf
//创建一个高斯概率密度函数
//有期望、协方差、还有分解后的矩阵、再产生一个随机值
pf_pdf_gaussian_t *pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx)
{
  pf_matrix_t cd;
  pf_pdf_gaussian_t *pdf;

  pdf = calloc(1, sizeof(pf_pdf_gaussian_t));

  pdf->x = x;//概率密度的期望
  pdf->cx = cx;//概率密度的协方差
  //pdf->cxi = pf_matrix_inverse(cx, &pdf->cxdet);

  // Decompose the convariance matrix into a rotation
  // matrix and a diagonal matrix.
  //他的分解是cx=cr*cd*cr'
  pf_matrix_unitary(&pdf->cr, &cd, pdf->cx);
  pdf->cd.v[0] = sqrt(cd.m[0][0]);
  pdf->cd.v[1] = sqrt(cd.m[1][1]);
  pdf->cd.v[2] = sqrt(cd.m[2][2]);

  // Initialize the random number generator
  //pdf->rng = gsl_rng_alloc(gsl_rng_taus);
  //gsl_rng_set(pdf->rng, ++pf_pdf_seed);
  //产生一个随机数种子值
  srand48(++pf_pdf_seed);

  return pdf;
}


// Destroy the pdf
void pf_pdf_gaussian_free(pf_pdf_gaussian_t *pdf)
{
  //gsl_rng_free(pdf->rng);
  free(pdf);
  return;
}


/*
// Compute the value of the pdf at some point [x].
double pf_pdf_gaussian_value(pf_pdf_gaussian_t *pdf, pf_vector_t x)
{
  int i, j;
  pf_vector_t z;
  double zz, p;
  
  z = pf_vector_sub(x, pdf->x);

  zz = 0;
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      zz += z.v[i] * pdf->cxi.m[i][j] * z.v[j];

  p =  1 / (2 * M_PI * pdf->cxdet) * exp(-zz / 2);
          
  return p;
}
*/


// Generate a sample from the pdf.
//
pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t *pdf)
{
  int i, j;
  pf_vector_t r;
  pf_vector_t x;

  // Generate a random vector
  //产生一个随机的向量
  //我的理解就是这个cd里面就是位置信息，因为他是有标准正态分布过来的
  //所以他的尺度就是位置信息
  //这个就是再位置信息附近产生一个值
  //后面再加上旋转的值
  for (i = 0; i < 3; i++)
  {
    //r.v[i] = gsl_ran_gaussian(pdf->rng, pdf->cd.v[i]);
    r.v[i] = pf_ran_gaussian(pdf->cd.v[i]);
  }
  for (i = 0; i < 3; i++)
  {
    x.v[i] = pdf->x.v[i];
    for (j = 0; j < 3; j++)
      x.v[i] += pdf->cr.m[i][j] * r.v[j];
  } 
  
  return x;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
// 从零均值高斯分布中随机抽取，具有标准偏差sigma。方差就是sigma^2
// 我们使用Box-Muller变换的极坐标形式，
 //pf_ran_gaussian函数实现了一个标准的高斯分布函数，其接收sigma参数
double pf_ran_gaussian(double sigma)
{
  double x1, x2, w, r;
  //drand48 返回服从均匀分布的·[0.0, 1.0) 之间的 double 型随机数
  do
  {
    do { r = drand48(); } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
}
