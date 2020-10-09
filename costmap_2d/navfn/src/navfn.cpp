/*********************************************************************
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
*********************************************************************/
//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
// 
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//全局规划中使用Dijkstra算法进行实际计算的部分在NavFn类里完成，
//它通过传入的costmap来设置costarr数组，
//再通过costarr数组对存储地图上所有cell点Potential值的potarr数组进行更新，
//并通过potarr数组来计算梯度gradX和gradY，通过迭代比较，
//最终得到一条完整的全局规划路径。



#include <navfn/navfn.h>
#include <ros/console.h>

namespace navfn {

  //
  // function to perform nav fn calculation
  // keeps track of internal buffers, will be more efficient
  //   if the size of the environment does not change
  //

  int
    create_nav_plan_astar(COSTTYPE *costmap, int nx, int ny,
        int* goal, int* start,
        float *plan, int nplan)
    {
      static NavFn *nav = NULL;

      if (nav == NULL)
        nav = new NavFn(nx,ny);

      if (nav->nx != nx || nav->ny != ny) // check for compatibility with previous call
      {
        delete nav;
        nav = new NavFn(nx,ny);      
      }

      nav->setGoal(goal);
      nav->setStart(start);

      nav->costarr = costmap;
      nav->setupNavFn(true);

      // calculate the nav fn and path
      nav->priInc = 2*COST_NEUTRAL;
      nav->propNavFnAstar(std::max(nx*ny/20,nx+ny));

      // path
      int len = nav->calcPath(nplan);

      if (len > 0)			// found plan
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
      else
        ROS_DEBUG("[NavFn] No path found\n");

      if (len > 0)
      {
        for (int i=0; i<len; i++)
        {
          plan[i*2] = nav->pathx[i];
          plan[i*2+1] = nav->pathy[i];
        }
      }

      return len;
    }




  //
  // create nav fn buffers 
  //

  NavFn::NavFn(int xs, int ys)
  {  
    // create cell arrays
    costarr = NULL;
    potarr = NULL;
    pending = NULL;
    gradx = grady = NULL;
    setNavArr(xs,ys);

    // priority buffers #define PRIORITYBUFSIZE 10000
    pb1 = new int[PRIORITYBUFSIZE];
    pb2 = new int[PRIORITYBUFSIZE];
    pb3 = new int[PRIORITYBUFSIZE];

    // for Dijkstra (breadth-first), set to COST_NEUTRAL
    // for A* (best-first), set to COST_NEUTRAL
    priInc = 2*COST_NEUTRAL;	

    // goal and start
    goal[0] = goal[1] = 0;
    start[0] = start[1] = 0;

    // display function
    displayFn = NULL;
    displayInt = 0;

    // path buffers
    npathbuf = npath = 0;
    pathx = pathy = NULL;
    pathStep = 0.5;
  }


  NavFn::~NavFn()
  {
    if(costarr)
      delete[] costarr;
    if(potarr)
      delete[] potarr;
    if(pending)
      delete[] pending;
    if(gradx)
      delete[] gradx;
    if(grady)
      delete[] grady;
    if(pathx)
      delete[] pathx;
    if(pathy)
      delete[] pathy;
    if(pb1)
      delete[] pb1;
    if(pb2)
      delete[] pb2;
    if(pb3)
      delete[] pb3;
  }


  //
  // set goal, start positions for the nav fn
  //

  void
    NavFn::setGoal(int *g)
    {
      goal[0] = g[0];
      goal[1] = g[1];
      ROS_DEBUG("[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
    }

  void
    NavFn::setStart(int *g)
    {
      start[0] = g[0];
      start[1] = g[1];
      ROS_DEBUG("[NavFn] Setting start to %d,%d\n", start[0], start[1]);
    }

  //
  // Set/Reset map size
  //
  //这个函数就是创建几个数组 costarr、potarr、pending、gradx、grady
  //costarr 记录全局costmap信息 就是costmap-array
  //potarr 储存各cell的Potential值，就是代价值表
  //pending 是否被遍历过
  //gradx、grady x和y向的梯度数组（用于生成路径）
  void
    NavFn::setNavArr(int xs, int ys)
    {
      ROS_DEBUG("[NavFn] Array is %d x %d\n", xs, ys);

      nx = xs;
      ny = ys;
      ns = nx*ny;

      if(costarr)
        delete[] costarr;
      if(potarr)
        delete[] potarr;
      if(pending)
        delete[] pending;

      if(gradx)
        delete[] gradx;
      if(grady)
        delete[] grady;
      //记录全局costmap信息 就是costmap-array
      costarr = new COSTTYPE[ns]; // cost array, 2d config space
      memset(costarr, 0, ns*sizeof(COSTTYPE));
      //储存各cell的Potential值
      potarr = new float[ns];	// navigation potential array
      pending = new bool[ns];
      memset(pending, 0, ns*sizeof(bool));
      //x和y向的梯度数组（用于生成路径）
      gradx = new float[ns];
      grady = new float[ns];
    }


  //这个函数的作用就是对一个costmap地图的每一个cell进行重新的赋值（50-254）
  //这个是对地图的更改，没有考虑路径或者说没有考虑目标及起点的事情

  // set up cost array, usually from ROS
  //costmap_->getCharMap()
  //将返回一个指针，指向作为costmap使用的底层无符号字符数组。
  //planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);
  // NavFn::setCostmap | 将Costmap“翻译”成costarr
  //这个是函数进行的第一步
  //就是将传入的costmap来设置costarr数组
  //cmap指向costmap元素，就是costmap中每一个cell所对应的值
  void
    NavFn::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown)
    {
      //就是将costarr地址给cm，costarr本身就是一个数组，
      //所以就是将首地址给cm
      COSTTYPE *cm = costarr;
      //cmap指向costmap元素，cm指向costarr
      //注：最小权重值即行走单个free(无障碍物影响)栅格所付出的权重代价  
      //最大权重值即行走单个障碍物栅格所付出的权重代价
      if (isROS)			// ROS-type cost array
      {
        for (int i=0; i<ny; i++)
        {
          int k=i*nx;
          for (int j=0; j<nx; j++, k++, cmap++, cm++)
          {
            // This transforms the incoming cost values:
            // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
            // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated obstacle")
            // values in range 0 to 252 -> values from COST_NEUTRAL to COST_OBS_ROS.
            //先把值全部设置为254
            //经过下面的判断，若都没有执行
            //也就是若当前cell在costmap上的值 == COST_OBS(254)，
            //即致命障碍物（障碍物本身），值仍为254。
            *cm = COST_OBS;
            //把costmap中的该cell单元的值给v
            int v = *cmap;
            //若当前cell在costmap上的值 < COST_OBS_ROS(253)，
            //即非致命障碍物（障碍物附近）------这个是重点
            //重新将其赋值为COST_NEUTRAL(50)+当前cell在costmap上的值×比例0.8，
            //最高253；
            if (v < COST_OBS_ROS)
            {
              v = COST_NEUTRAL+COST_FACTOR*v;
              //这个就是防止其超越界限
              if (v >= COST_OBS)
                v = COST_OBS-1;
              *cm = v;
            }
            //若当前cell在costmap上的值 == COST_UNKNOWN_ROS(255)，
            //即未知区域，赋值为253；
            else if(v == COST_UNKNOWN_ROS && allow_unknown)
            {
              v = COST_OBS-1;
              *cm = v;
            }
          }
        }
      }
      //当地图类型是其他类型（如PGM），
      //也执行同样的“翻译”工作，设置costarr数组。
      else				// not a ROS map, just a PGM
      {
        for (int i=0; i<ny; i++)
        {
          int k=i*nx;
          for (int j=0; j<nx; j++, k++, cmap++, cm++)
          {
            *cm = COST_OBS;
            if (i<7 || i > ny-8 || j<7 || j > nx-8)
              continue;	// don't do borders
            int v = *cmap;
            if (v < COST_OBS_ROS)
            {
              v = COST_NEUTRAL+COST_FACTOR*v;
              if (v >= COST_OBS)
                v = COST_OBS-1;
              *cm = v;
            }
            else if(v == COST_UNKNOWN_ROS)
            {
              v = COST_OBS-1;
              *cm = v;
            }
          }
        }

      }
    }



  //NavFn::calcNavFnDijkstra | Dijkstra计算主体部分
  //这个函数内完成了整个路径计算的流程，顺序调用了几个子部分的函数。
  bool
    NavFn::calcNavFnDijkstra(bool atStart)
    {
      //重新设置势场矩阵potarr的值、设置costarr的边际值、
      //设置目标在costarr中的值为0，
      //对四周cell进行处理，记录costarr中障碍物cell数
      setupNavFn(true);

      // calculate the nav fn and path
      //这个函数以目标点（Potential值已初始化为0）为起点，
      //向整张地图的cell传播，填充potarr数组，
      //直到找到起始点为止，
      //potarr数组的数据能够反映“走了多远”和“附近的障碍情况”，
      //为最后的路径计算提供了依据。
      propNavFnDijkstra(std::max(nx*ny/20,nx+ny),atStart);

      // path
      //该函数负责在potarr数组的基础上选取一些cell点来生成最终的全局规划路径，
      //从起点开始沿着最优行走代价值梯度下降的方向寻找到目标点的最优轨迹。
      int len = calcPath(nx*ny/2);

      if (len > 0)			// found plan
      {
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
        return true;
      }
      else
      {
        ROS_DEBUG("[NavFn] No path found\n");
        return false;
      }

    }


  //
  // calculate navigation function, given a costmap, goal, and start
  //

  bool
    NavFn::calcNavFnAstar()
    {
      setupNavFn(true);

      // calculate the nav fn and path
      propNavFnAstar(std::max(nx*ny/20,nx+ny));

      // path
      int len = calcPath(nx*4);

      if (len > 0)			// found plan
      {
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
        return true;
      }
      else
      {
        ROS_DEBUG("[NavFn] No path found\n");
        return false;
      }
    }


  //
  // returning values
  //

  float *NavFn::getPathX() { return pathx; }
  float *NavFn::getPathY() { return pathy; }
  int    NavFn::getPathLen() { return npath; }

  // inserting onto the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && curPe<PRIORITYBUFSIZE) \
  { curP[curPe++]=n; pending[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && nextPe<PRIORITYBUFSIZE) \
  { nextP[nextPe++]=n; pending[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && overPe<PRIORITYBUFSIZE) \
  { overP[overPe++]=n; pending[n]=true; }}


  // Set up navigation potential arrays for new propagation
  // NavFn::setupNavFn | 处理costarr，初始化potarr、gradx、grady
  //该函数对“翻译”生成的costarr数组进行了边际设置等处理，
  //并初始化了potarr数组就是将目标的代价值设置为0
  //和创建了openlist和closedlist，并且把目标四周的点放到了curP中
  void
    NavFn::setupNavFn(bool keepit)
    {
      // reset values in propagation arrays
      //重新设置势场矩阵potarr的值
      for (int i=0; i<ns; i++)
      {
        //将代价值表初始化为最大值，默认起点到所有点的行走代价值都为最大
        potarr[i] = POT_HIGH;
        if (!keepit) costarr[i] = COST_NEUTRAL;
        gradx[i] = grady[i] = 0.0;
      }

      // outer bounds of cost array
      COSTTYPE *pc;
      pc = costarr;
      //costarr第一行全部设置为COST_OBS(致命层254)
      for (int i=0; i<nx; i++)
        *pc++ = COST_OBS;
      //costarr最后一行全部设置为COST_OBS(致命层254)
      pc = costarr + (ny-1)*nx;
      for (int i=0; i<nx; i++)
        *pc++ = COST_OBS;
      //costarr第一列全部设置为COST_OBS(致命层254)
      pc = costarr;
      for (int i=0; i<ny; i++, pc+=nx)
        *pc = COST_OBS;
      //costarr最后一列全部设置为COST_OBS(致命层254)
      pc = costarr + nx - 1;
      for (int i=0; i<ny; i++, pc+=nx)
        *pc = COST_OBS;

      // priority buffers
      //当前的阀值
      curT = COST_OBS;
      //当前用于传播的cell索引数组
      curP = pb1; 
      //当前用于传播的cell的数量
      curPe = 0;
      //用于下个传播过程的cell索引数组
      nextP = pb2;
      //用于下个传播过程的cell的数量
      nextPe = 0;
      ////传播界限外的cell索引数组
      overP = pb3;
      //传播界限外的cell的数量
      overPe = 0;
      //并初始化pending数组为全0，设置所有的cell状态都为非等待状态。
      memset(pending, 0, ns*sizeof(bool));

      // set goal这里的goal就是开始点
      int k = goal[0] + goal[1]*nx;
      //接下来设置目标goal在potarr中的值为0，（就是设置开始点是0）
      //并把它四周非障碍物的cell加入curP数组（当前传播cell）中，
      //为下一步的Potential值在整张地图上的传播做准备。
      //这个函数作用见下面描述
      initCost(k,0);

      // find # of obstacle cells
      //更新nobs，记录costarr中的致命障碍物cell的数量
      pc = costarr;
      int ntot = 0;
      for (int i=0; i<ns; i++, pc++)
      {
        if (*pc >= COST_OBS)
          ntot++;			// number of cells that are obstacles
      }
      nobs = ntot;
    }


  // initialize a goal-type cost for starting propagation
  //这个函数就是这个作用，push_cur会判断：将不是障碍物，没有被遍历过
  //并且curP没满，九八这个值放进去，并把对应位的pending值设置为true
  void
    NavFn::initCost(int k, float v)
    {
      potarr[k] = v;
      push_cur(k+1);
      push_cur(k-1);
      push_cur(k-nx);
      push_cur(k+nx);
    }


  // 
  // Critical function: calculate updated potential value of a cell,
  //   given its neighbors' values
  // Planar-wave update calculation from two lowest neighbors in a 4-grid
  // Quadratic approximation to the interpolated value 
  // No checking of bounds here, this function should be fast
  //

#define INVSQRT2 0.707106781
  //NavFn::updateCell | 单点Potential传播
  //updateCell用于更新单个cell的Potential值，
  //先获取当前cell四周邻点的potarr值，并取最小的值存入ta。
  //并将其四周符合特定条件的点放入nextP或overP，
  //用于下一步的传播。
  inline void
    NavFn::updateCell(int n)
    {
      // get neighbors
      float u,d,l,r;
      l = potarr[n-1];
      r = potarr[n+1];		
      u = potarr[n-nx];
      d = potarr[n+nx];
      //  ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n", 
      //	 potarr[n], l, r, u, d);
      //  ROS_INFO("[Update] cost: %d\n", costarr[n]);

      // find lowest, and its lowest neighbor
      float ta, tc;
      if (l<r) tc=l; else tc=r;
      if (u<d) ta=u; else ta=d;

      // do planar wave update
      //下面执行一个判断，只有当当前cell不是致命障碍物时，
      //才由它向四周传播，否则到它后停止，不传播。
      if (costarr[n] < COST_OBS)	// don't propagate into obstacles
      {
        float hf = (float)costarr[n]; // traversability factor
        float dc = tc-ta;		// relative cost between ta,tc
        if (dc < 0) 		//   tc<ta  说明tc最低
        {
          dc = -dc;
          ta = tc; //把tc值给ta，使得ta是最低的值
        }

        // calculate new potential
        //这个只需要知道pot就是从四周cell中最小的potarr到本cell的potarr
        float pot;
        if (dc >= hf)		// if too large, use ta-only update
          pot = ta+hf;
        else			// two-neighbor interpolation update
        {
          // use quadratic approximation
          // might speed this up through table lookup, but still have to 
          //   do the divide
          float d = dc/hf;
          float v = -0.2301*d*d + 0.5307*d + 0.7040;
          pot = ta + hf*v;
        }

        //      ROS_INFO("[Update] new pot: %d\n", costarr[n]);

        // now add affected neighbors to priority blocks
        if (pot < potarr[n])
        {
          //#define INVSQRT2 0.707106781
          //不懂这个
          float le = INVSQRT2*(float)costarr[n-1];
          float re = INVSQRT2*(float)costarr[n+1];
          float ue = INVSQRT2*(float)costarr[n-nx];
          float de = INVSQRT2*(float)costarr[n+nx];
          potarr[n] = pot;
          if (pot < curT)	// low-cost buffer block 
          {
            if (l > pot+le) push_next(n-1);
            if (r > pot+re) push_next(n+1);
            if (u > pot+ue) push_next(n-nx);
            if (d > pot+de) push_next(n+nx);
          }
          else			// overflow block
          {
            if (l > pot+le) push_over(n-1);
            if (r > pot+re) push_over(n+1);
            if (u > pot+ue) push_over(n-nx);
            if (d > pot+de) push_over(n+nx);
          }
        }

      }

    }


  //
  // Use A* method for setting priorities
  // Critical function: calculate updated potential value of a cell,
  //   given its neighbors' values
  // Planar-wave update calculation from two lowest neighbors in a 4-grid
  // Quadratic approximation to the interpolated value 
  // No checking of bounds here, this function should be fast
  //

#define INVSQRT2 0.707106781

  inline void
    NavFn::updateCellAstar(int n)
    {
      // get neighbors
      float u,d,l,r;
      l = potarr[n-1];
      r = potarr[n+1];		
      u = potarr[n-nx];
      d = potarr[n+nx];
      //ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n", 
      //	 potarr[n], l, r, u, d);
      // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

      // find lowest, and its lowest neighbor
      float ta, tc;
      if (l<r) tc=l; else tc=r;
      if (u<d) ta=u; else ta=d;

      // do planar wave update
      if (costarr[n] < COST_OBS)	// don't propagate into obstacles
      {
        float hf = (float)costarr[n]; // traversability factor
        float dc = tc-ta;		// relative cost between ta,tc
        if (dc < 0) 		// ta is lowest
        {
          dc = -dc;
          ta = tc;
        }

        // calculate new potential
        float pot;
        if (dc >= hf)		// if too large, use ta-only update
          pot = ta+hf;
        else			// two-neighbor interpolation update
        {
          // use quadratic approximation
          // might speed this up through table lookup, but still have to 
          //   do the divide
          float d = dc/hf;
          float v = -0.2301*d*d + 0.5307*d + 0.7040;
          pot = ta + hf*v;
        }

        //ROS_INFO("[Update] new pot: %d\n", costarr[n]);

        // now add affected neighbors to priority blocks
        if (pot < potarr[n])
        {
          float le = INVSQRT2*(float)costarr[n-1];
          float re = INVSQRT2*(float)costarr[n+1];
          float ue = INVSQRT2*(float)costarr[n-nx];
          float de = INVSQRT2*(float)costarr[n+nx];

          // calculate distance
          int x = n%nx;
          int y = n/nx;
          float dist = hypot(x-start[0], y-start[1])*(float)COST_NEUTRAL;

          potarr[n] = pot;
          pot += dist;
          if (pot < curT)	// low-cost buffer block 
          {
            if (l > pot+le) push_next(n-1);
            if (r > pot+re) push_next(n+1);
            if (u > pot+ue) push_next(n-nx);
            if (d > pot+de) push_next(n+nx);
          }
          else
          {
            if (l > pot+le) push_over(n-1);
            if (r > pot+re) push_over(n+1);
            if (u > pot+ue) push_over(n-nx);
            if (d > pot+de) push_over(n+nx);
          }
        }

      }

    }



  //
  // main propagation function
  // Dijkstra method, breadth-first 广度优先搜索
  // runs for a specified number of cycles,
  //   or until it runs out of cells to update,
  //   or until the Start cell is found (atStart = true)
  //
  // NavFn::propNavFnDijkstra | 更新potarr数组
  //这个函数以目标点（Potential值已初始化为0）为起点，
  //向整张地图的cell传播，填充potarr数组，
  //直到找到起始点为止，
  //potarr数组的数据能够反映“走了多远”和“附近的障碍情况”，
  //为最后的路径计算提供了依据。
  bool
    NavFn::propNavFnDijkstra(int cycles, bool atStart)	
    {
      //priority block的最大数量
      int nwv = 0;			// max priority block size
      //priority blocks中的cell数
      int nc = 0;			// number of cells put into priority blocks
      //当前迭代次数
      int cycle = 0;		// which cycle we're on

      // set up start cell
      //这里的start是planner_->setStart(map_goal);
      //是目标值
      int startCell = start[1]*nx + start[0];
      //没懂这个cycles
      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {
        //循环迭代更新potarr，判断条件：如果当前正在传播和下一步传播的集都为空，
        //那么说明已经无法继续传播，可能有无法越过的障碍或其他情况，退出。
        if (curPe == 0 && nextPe == 0) // priority blocks empty
          break;

        // stats
        //记录的就是进入过curP的数量
        nc += curPe;
        if (curPe > nwv)
          nwv = curPe;

        // reset pending flags on current priority buffer
        //对pending数组进行设置
        int *pb = curP;
        int i = curPe;	
        //这个地方是要弄成false的
        //因为别的cell的附近也可能有这些cell，所以这些cell还是可以放进curP中		
        while (i-- > 0)		
          pending[*(pb++)] = false;

        // process current priority buffer
        //接下来传播curP，即当前cell，调用的函数为updateCell，
        //它能更新当前cell在potarr数组中的值，
        //并将其四周符合特定条件的点放入nextP或overP，
        //用于下一步的传播。调用完成后，将nextP数组中的cell传递给curP，
        //继续上述传播，
        //若nextP没有cell可以用来传播，则引入overP中的cell。
        pb = curP; 
        i = curPe;
        while (i-- > 0)		
          updateCell(*pb++);

        if (displayInt > 0 &&  (cycle % displayInt) == 0)
          displayFn(this);

        // swap priority blocks curP <=> nextP
        curPe = nextPe;
        nextPe = 0;
        pb = curP;		// swap buffers
        curP = nextP;
        nextP = pb;

        // see if we're done with this priority level
        if (curPe == 0)
        {
          //这里有个问题，就是当有一次用的是overP里面的数据
          //而下一次用的是nestP中的数据，那么curT是没有更换回来的
          curT += priInc;	// increment priority threshold增加优先级阈值
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // check if we've hit the Start cell
        //在从目标点向全地图传播的过程中检查，
        //当起点的Potential值不再是被初始化的无穷大，
        //而是有一个实际的值时，说明到达了起点，传播停止。
        if (atStart)
          if (potarr[startCell] < POT_HIGH)
            break;
      }

      ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n", 
          cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);

      if (cycle < cycles) return true; // finished up here
      else return false;
    }


  //
  // main propagation function
  // A* method, best-first
  // uses Euclidean distance heuristic
  // runs for a specified number of cycles,
  //   or until it runs out of cells to update,
  //   or until the Start cell is found (atStart = true)
  //

  bool
    NavFn::propNavFnAstar(int cycles)	
    {
      int nwv = 0;			// max priority block size
      int nc = 0;			// number of cells put into priority blocks
      int cycle = 0;		// which cycle we're on

      // set initial threshold, based on distance
      float dist = hypot(goal[0]-start[0], goal[1]-start[1])*(float)COST_NEUTRAL;
      curT = dist + curT;

      // set up start cell
      int startCell = start[1]*nx + start[0];

      // do main cycle
      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {
        // 
        if (curPe == 0 && nextPe == 0) // priority blocks empty
          break;

        // stats
        nc += curPe;
        if (curPe > nwv)
          nwv = curPe;

        // reset pending flags on current priority buffer
        int *pb = curP;
        int i = curPe;			
        while (i-- > 0)		
          pending[*(pb++)] = false;

        // process current priority buffer
        pb = curP; 
        i = curPe;
        while (i-- > 0)		
          updateCellAstar(*pb++);

        if (displayInt > 0 &&  (cycle % displayInt) == 0)
          displayFn(this);

        // swap priority blocks curP <=> nextP
        curPe = nextPe;
        nextPe = 0;
        pb = curP;		// swap buffers
        curP = nextP;
        nextP = pb;

        // see if we're done with this priority level
        if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // check if we've hit the Start cell
        if (potarr[startCell] < POT_HIGH)
          break;

      }

      last_path_cost_ = potarr[startCell];

      ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n", 
          cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);


      if (potarr[startCell] < POT_HIGH) return true; // finished up here
      else return false;
    }


  float NavFn::getLastPathCost()
  {
    return last_path_cost_;
  }


  //
  // Path construction
  // Find gradient at array points, interpolate path
  // Use step size of pathStep, usually 0.5 pixel
  //
  // Some sanity checks:
  //  1. Stuck at same index position
  //  2. Doesn't get near goal
  //  3. Surrounded by high potentials
  //
  // NavFn::calcPath | 路径生成
  //该函数负责在potarr数组的基础上选取一些cell点来生成最终的全局规划路径，
  //从起点开始沿着最优行走代价值梯度下降的方向寻找到目标点的最优轨迹。
  int
    NavFn::calcPath(int n, int *st)
    {
      // test write
      //savemap("test");

      // check path arrays
      if (npathbuf < n)
      {
        if (pathx) delete [] pathx;
        if (pathy) delete [] pathy;
        pathx = new float[n];
        pathy = new float[n];
        npathbuf = n;
      }

      // set up start position at cell
      // st is always upper left corner for 4-point bilinear interpolation
      // 对于4点双线性插值，st总是左上角 
      // 这里的start就是我们的目标点
      //目标点是potarr最大的值，从这里沿着梯度下降方向进行寻找
      if (st == NULL) st = start;
      //stc记录起点索引
      int stc = st[1]*nx + st[0];

      // set up offset
      // 设置偏移量
      float dx=0;
      float dy=0;
      npath = 0;//路径点索引

      // go for <n> cycles at most
      for (int i=0; i<n; i++)
      {
        // check if near goal
        //检查是否接近目标
        int nearest_point=std::max(0,std::min(nx*ny-1,stc+(int)round(dx)+(int)(nx*round(dy))));
        if (potarr[nearest_point] < COST_NEUTRAL)
        {
          pathx[npath] = (float)goal[0];
          pathy[npath] = (float)goal[1];
          return ++npath;	// done!
        }
        //在第一行或最后一行，即超出边界
        if (stc < nx || stc > ns-nx) // would be out of bounds
        {
          ROS_DEBUG("[PathCalc] Out of bounds");
          return 0;
        }

        // add to path
        //添加至路径点
        //把目标点添加至路径
        pathx[npath] = stc%nx + dx;
        pathy[npath] = stc/nx + dy;
        npath++;

        bool oscillation_detected = false;
        //震荡检测，某一步和上上一步的位置是否一样
        if( npath > 2 &&
            pathx[npath-1] == pathx[npath-3] &&
            pathy[npath-1] == pathy[npath-3] )
        {
          ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
          oscillation_detected = true;
        }
        //当前点下方的点的索引
        int stcnx = stc+nx;
        //当前点上方的点的索引
        int stcpx = stc-nx;

        // check for potentials at eight positions near cell
        //检查当前到达节点的周边的8个节点是否有障碍物代价值或者再震荡，
        //如果有的话，则直接将stc指向这8个节点中potential值最低的节点
        if (potarr[stc] >= POT_HIGH ||
            potarr[stc+1] >= POT_HIGH ||
            potarr[stc-1] >= POT_HIGH ||
            potarr[stcnx] >= POT_HIGH ||
            potarr[stcnx+1] >= POT_HIGH ||
            potarr[stcnx-1] >= POT_HIGH ||
            potarr[stcpx] >= POT_HIGH ||
            potarr[stcpx+1] >= POT_HIGH ||
            potarr[stcpx-1] >= POT_HIGH ||
            oscillation_detected)
        {
          ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);
          // check eight neighbors to find the lowest
          int minc = stc;
          int minp = potarr[stc];
          int st = stcpx - 1;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st = stc-1;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st = stc+1;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st = stcnx-1;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          stc = minc;
          dx = 0;
          dy = 0;

          ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
              potarr[stc], pathx[npath-1], pathy[npath-1]);

          if (potarr[stc] >= POT_HIGH)
          {
            ROS_DEBUG("[PathCalc] No path found, high potential");
            //savemap("navfn_highpot");
            return 0;
          }
        }

        // have a good gradient here
        //如果有好的梯度，则直接计算梯度，并沿着梯度方向查找下一个节点
        else			
        {

          // get grad at four positions near cell
          // 在单元格附近的四个位置获得grad
          gradCell(stc);
          gradCell(stc+1);
          gradCell(stcnx);
          gradCell(stcnx+1);


          // get interpolated gradient
          //得到插值梯度
          float x1 = (1.0-dx)*gradx[stc] + dx*gradx[stc+1];
          float x2 = (1.0-dx)*gradx[stcnx] + dx*gradx[stcnx+1];
          float x = (1.0-dy)*x1 + dy*x2; // interpolated x
          float y1 = (1.0-dx)*grady[stc] + dx*grady[stc+1];
          float y2 = (1.0-dx)*grady[stcnx] + dx*grady[stcnx+1];
          float y = (1.0-dy)*y1 + dy*y2; // interpolated y

          // show gradients
          ROS_DEBUG("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
                    gradx[stc], grady[stc], gradx[stc+1], grady[stc+1], 
                    gradx[stcnx], grady[stcnx], gradx[stcnx+1], grady[stcnx+1],
                    x, y);

          // check for zero gradient, failed
          if (x == 0.0 && y == 0.0)
          {
            ROS_DEBUG("[PathCalc] Zero gradient");	  
            return 0;
          }

          // move in the right direction
          // 朝着正确的方向发展
          float ss = pathStep/hypot(x, y);
          dx += x*ss;
          dy += y*ss;

          // check for overflow
          //溢出检查
          if (dx > 1.0) { stc++; dx -= 1.0; }
          if (dx < -1.0) { stc--; dx += 1.0; }
          if (dy > 1.0) { stc+=nx; dy -= 1.0; }
          if (dy < -1.0) { stc-=nx; dy += 1.0; }

        }

        //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //	     potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
      }

      //  return npath;			// out of cycles, return failure
      ROS_DEBUG("[PathCalc] No path found, path too long");
      //savemap("navfn_pathlong");
      return 0;			// out of cycles, return failure
    }


  //
  // gradient calculations
  //

  // calculate gradient at a cell
  // positive value are to the right and down正值是向右和向下的
  //返回这个点下的梯度
  float				
    NavFn::gradCell(int n)
    {
      if (gradx[n]+grady[n] > 0.0)	// check this cell
        return 1.0;			

      if (n < nx || n > ns-nx)	// would be out of bounds
        return 0.0;

      float cv = potarr[n];
      float dx = 0.0;
      float dy = 0.0;

      // check for in an obstacle
      if (cv >= POT_HIGH) 
      {
        if (potarr[n-1] < POT_HIGH)
          dx = -COST_OBS;
        else if (potarr[n+1] < POT_HIGH)
          dx = COST_OBS;

        if (potarr[n-nx] < POT_HIGH)
          dy = -COST_OBS;
        else if (potarr[n+nx] < POT_HIGH)
          dy = COST_OBS;
      }

      else				// not in an obstacle
      {
        // dx calc, average to sides
        if (potarr[n-1] < POT_HIGH)
          dx += potarr[n-1]- cv;	
        if (potarr[n+1] < POT_HIGH)
          dx += cv - potarr[n+1]; 

        // dy calc, average to sides
        if (potarr[n-nx] < POT_HIGH)
          dy += potarr[n-nx]- cv;	
        if (potarr[n+nx] < POT_HIGH)
          dy += cv - potarr[n+nx]; 
      }

      // normalize
      //归一化
      float norm = hypot(dx, dy);
      if (norm > 0)
      {
        norm = 1.0/norm;
        gradx[n] = norm*dx;
        grady[n] = norm*dy;
      }
      return norm;
    }


  //
  // display function setup
  // <n> is the number of cycles to wait before displaying,
  //     use 0 to turn it off

  void
    NavFn::display(void fn(NavFn *nav), int n)
    {
      displayFn = fn;
      displayInt = n;
    }


  //
  // debug writes
  // saves costmap and start/goal
  //

  void 
    NavFn::savemap(const char *fname)
    {
      char fn[4096];

      ROS_DEBUG("[NavFn] Saving costmap and start/goal points");
      // write start and goal points
      sprintf(fn,"%s.txt",fname);
      FILE *fp = fopen(fn,"w");
      if (!fp)
      {
        ROS_WARN("Can't open file %s", fn);
        return;
      }
      fprintf(fp,"Goal: %d %d\nStart: %d %d\n",goal[0],goal[1],start[0],start[1]);
      fclose(fp);

      // write cost array
      if (!costarr) return;
      sprintf(fn,"%s.pgm",fname);
      fp = fopen(fn,"wb");
      if (!fp)
      {
        ROS_WARN("Can't open file %s", fn);
        return;
      }
      fprintf(fp,"P5\n%d\n%d\n%d\n", nx, ny, 0xff);
      fwrite(costarr,1,nx*ny,fp);
      fclose(fp);
    }
};
