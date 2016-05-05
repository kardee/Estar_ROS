/*
/* ROS-ESTAR.
 *
 * Copyright (C) 2016 KARTHIKEYAN. All rights reserved.
 * License (3-Cluase BSD): https://github.com/kardee/Estar_ROS
 *
 * This code uses and is based on code from:
 *   Project: ESTAR_ROS https://https://github.com/poftwaresatent/estar2
 *   Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *   License (3-Clause BSD) : https://github.com/poftwaresatent/estar2
 * ***/
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <string>

/** including ros libraries**********************/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
/** ********************************************/ 
#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

//#include <pcl_conversions/pcl_conversions.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


#define ODIST 3
static int drag,have_goal,mx,my;

#ifndef ESTARROS_H
#define ESTARROS_H
#define CALC_KEY(cell) ((cell)->rhs < (cell)->phi ? (cell)->rhs : (cell)->phi)
#define estar_grid_at(grid,ix,iy) (&(grid)->cell[(ix)+(iy)*(grid)->dimx])

using namespace std;

enum 
{
  ESTAR_FLAG_GOAL     = 1,
  ESTAR_FLAG_OBSTACLE = 2
};


typedef struct estar_cell_s 
{
  double cost;			 /* set this to 1/speed for "sensible" values */
  double phi;
  double rhs;
  double key;			 /* managed by pqueue */
  size_t pqi;			 /* managed by pqueue; pqi==0 means "not on queue" */
  int flags;
  struct estar_cell_s * nbor[5]; /* null-terminated array of neighbors */
  struct estar_cell_s * prop[9]; /* null-terminated array of pairwise propagators */
} estar_cell_t;

typedef struct 
{
  estar_cell_t * cell;
  size_t dimx, dimy;
} estar_grid_t;

typedef struct 
{
  estar_cell_t ** heap;
  size_t len, cap;
} estar_pqueue_t;

typedef struct 
{
  estar_grid_t grid;   // estar_grid_t is structure included in grid.h
  estar_pqueue_t pq;   // estar_pqueue_t is structure included in pqueue.h
} estar_t;

namespace Estar_planner
{
    class EstarROS: public nav_core::BaseGlobalPlanner 
    {
        public:
		EstarROS();
		EstarROS(string name, costmap_2d::Costmap2DROS* costmap_ros);
		ros::NodeHandle ROSNodeHandle;

		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
		bool makePlan(const geometry_msgs::PoseStamped& start, 
		              const geometry_msgs::PoseStamped& goal, 
		              std::vector<geometry_msgs::PoseStamped>& plan);			    
		~EstarROS();
        protected:
        private:
		void Estar_init(estar_t *estar,size_t dimx,size_t dimy);
		void Estar_grid_init(estar_grid_t * grid, size_t dimx, size_t dimy);
		void Estar_pqueue_fini (estar_pqueue_t * pq);
		void Estar_grid_fini (estar_grid_t * grid);
		void Estar_pqueue_init(estar_pqueue_t * pq, size_t cap);
		void Estar_set_speed (estar_t *estar, size_t ix, size_t iy, double speed);
		void Estar_update(estar_t * estar, estar_cell_t * cell);
		void Estar_pqueue_insert_or_update (estar_pqueue_t * pq, estar_cell_t * cell);
		void Estar_pqueue_remove_or_ignore (estar_pqueue_t * pq, estar_cell_t * cell);
		void Estar_propagate (estar_t *estar);
		void Estar_set_goal (estar_t * estar, size_t ix, size_t iy);
		void Estar_dump_queue (estar_t * estar, char const * pfx);
		int  Estar_check (estar_t * estar, char const * pfx);
		int  Estar_cell_calc_gradient (estar_cell_t * cell, double * gx, double * gy);		
		void swap (estar_cell_t ** aa, estar_cell_t ** bb);
		void bubble_up (estar_cell_t ** heap, size_t index);
		void bubble_down (estar_cell_t ** heap, size_t len, size_t index);
		double Estar_pqueue_topkey (estar_pqueue_t * pq);
		void calc_rhs (estar_cell_t * cell, double phimax);
		double interpolate (double cost, double primary, double secondary);
		estar_cell_t * Estar_pqueue_extract (estar_pqueue_t * pq);
		void change_obstacle (int cx, int cy, int dist, int add);
		void Estar_fini (estar_t * estar);

		estar_t estar;
		float originX;
		float originY;
		float resolution;
		costmap_2d::Costmap2DROS* costmap_ros_;
		costmap_2d::Costmap2D* costmap_;
		bool initialized_;
		int width;
		int height;
		int mapSize;

    };
};

#endif // ESTARROS_H
