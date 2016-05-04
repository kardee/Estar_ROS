#include "EstarROS.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(Estar_planner::EstarROS, nav_core::BaseGlobalPlanner)

float tBreak;
int value;
namespace Estar_planner
{
EstarROS::EstarROS()
{
    //ctor

}
EstarROS::EstarROS(string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	initialize(name,costmap_ros);
}

void EstarROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	if(!initialized_)
	{
		costmap_ros_ = costmap_ros;
		costmap_ = costmap_ros_->getCostmap();

		ros::NodeHandle private_nh("~/" + name);

		originX = costmap_->getOriginX();
		originY = costmap_->getOriginY();
		
		width = costmap_->getSizeInCellsX();
		height = costmap_->getSizeInCellsY();

		resolution = costmap_->getResolution();
		mapSize = width * height;
		
 		tBreak= 1+1/(mapSize);
		value = 0;
		Estar_init(&estar,width,height);

		for(int i=0;i<width;++i)
		{	
			for(int j=0;j<height;++j)
			{
				size_t cost = static_cast<size_t>(costmap_->getCost(i,j));
				if(cost == 0)
					Estar_set_speed(&estar,i,j,0.0);
				else
					Estar_set_speed(&estar,i,j,1.0);
			}
		}
		
		ROS_INFO("Estar planner initialized successfully.");
		initialized_ = true;
	}
	else
	{	
		ROS_WARN("This planner has allready been initialized..doing nothing");
	}
		
}

bool EstarROS::makePlan(const geometry_msgs::PoseStamped& start, 
		              const geometry_msgs::PoseStamped& goal, 
		              std::vector<geometry_msgs::PoseStamped>& plan)
{
	if(!initialized_)
	{
		ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner.");
		return false;
	}
	ROS_DEBUG("Got a start : %.2f, %.2f and a goal : %.2f, %.2f",start.pose.position.x,
								     start.pose.position.y,
								     goal.pose.position.x,
								     goal.pose.position.y);
	plan.clear();
	
	if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
	{	
		ROS_ERROR("This planner as configured will only accept goals in the %s frame,but a goal was sent in the %s frame.",
			   costmap_ros_->getGlobalFrameID().c_str(),goal.header.frame_id.c_str());
		return false;
	}
	
	tf::Stamped < tf::Pose > goal_tf;
	tf::Stamped < tf::Pose > start_tf;

	poseStampedMsgToTF(goal,goal_tf);
	poseStampedMsgToTF(start,start_tf);

	size_t startX = start.pose.position.x;
	size_t startY = start.pose.position.y;

	size_t goalX = goal.pose.position.x;
	size_t goalY = goal.pose.position.y;

	ROS_INFO("Setting E* goal to %2d %2d",goalX,goalY);
	Estar_set_goal(&estar,goalX,goalY);
	have_goal = 1;
	
	while(estar.pq.len != 0)
	{	
		Estar_propagate (&estar);;
	}
	ROS_INFO("Tracing back from start %2d %2d",startX,startY);
	
	size_t ii, jj;
	double topkey, maxknown, maxoverall;
        estar_cell_t * cell;
	topkey = Estar_pqueue_topkey (&estar.pq);
  	maxknown = 0.0;
 	maxoverall = 0.0;
 	for (ii = 0; ii < width; ++ii)
 	{
	  	for (jj = 0; jj < height; ++jj)
  	  	{
   	 		cell = estar_grid_at (&estar.grid, ii, jj);
  	        	if (cell->rhs == cell->phi && isfinite(cell->rhs)) 
  	       		{
	           		if (0 == cell->pqi && cell->rhs <= topkey && maxknown < cell->rhs) 
                    		{
	                		maxknown = cell->rhs;
	            		}
	            		if (maxoverall < cell->rhs)
                    		{
	               			maxoverall = cell->rhs;
	            		}
                	}	 
          	}
        }
        
        if (maxknown == 0.0) 
        {    
		maxknown = 0.0001;
        }
        if (maxoverall == 0.0) 
        {
		maxoverall = 0.0001;
        }
	
	cell = estar_grid_at(&estar.grid, startX,startY);
	if(cell->pqi == 0 && cell->rhs <= maxknown)
	{
		double px,py,dd,dmax,ds;
		px = startX;
		py = startY;

		dmax = 1.3 * cell->rhs;
		ds = 0.1;
		for(dd = 0.0; dd <= dmax; dd = dd + ds)
		{
			double gx,gy,gg;
			
			if(0 == Estar_cell_calc_gradient(cell,&gx,&gy));
			{	
				break;
			}
			gg = sqrt(pow(gx,2.0) + pow(gy,2.0));
			gx = gx * ds/gg;
			gy = gy * ds/gg;
			
			px = px + gx;
			py = py + gy;
			
			geometry_msgs::PoseStamped pose = goal;
	
			pose.pose.position.x = px;
			pose.pose.position.y = py;
			pose.pose.position.z = 0.0;
			
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 0.0;
			
			size_t ix = (size_t) rint (px);
			size_t iy = (size_t) rint (py);
	
			if(ix < 0 || ix >= width || iy < 0 || iy >= height)
			{
				ROS_INFO("Fell off the map at %2d %2d", ix,iy);
				break;
			}
			cell = estar_grid_at(&estar.grid,ix,iy);
			if(cell->flags & ESTAR_FLAG_GOAL)
			{
				ROS_INFO("Hit the goal at %2d %2d",ix,iy);
				break;
			}
		}		
	}
	return true;
}
EstarROS::~EstarROS()
{
	Estar_fini(&estar);
}

void EstarROS::Estar_fini(estar_t * estar)
{
	 Estar_pqueue_fini (&estar->pq);
 	 Estar_grid_fini (&estar->grid);
}

void  EstarROS::Estar_init(estar_t *estar,size_t dimx,size_t dimy)
{
	Estar_grid_init(&estar->grid,dimx,dimy);
	Estar_pqueue_init(&estar->pq,dimx+dimy);
}

void EstarROS::Estar_pqueue_init(estar_pqueue_t * pq, size_t cap)
{
	pq->heap = (estar_cell_t**) malloc (sizeof(estar_cell_t**) * (cap+1));
	pq->len = 0;
	pq->cap = cap;	
}

void EstarROS::Estar_grid_init(estar_grid_t * grid, size_t dimx, size_t dimy)
{
	size_t x,y;
	estar_cell_t *cell;
	estar_cell_t **nbor;
	size_t a = sizeof(estar_cell_t);
	grid->cell =(estar_cell_t *) malloc (a * dimx * dimy);
	
	grid->dimx = dimx;
        grid->dimy = dimy;
	int sum =0;
	for (x = 0; x < dimx; ++x)
        {
    		for (y = 0; y < dimy; ++y) 
		{
			//++sum;
			//cout << "Allocated memory " << sum << endl;
      			cell = estar_grid_at(grid, x, y);
      			cell->cost = 1.0;
           		cell->phi = INFINITY;
            		cell->rhs = INFINITY;
            		cell->key = INFINITY;
            		cell->pqi = 0;
            		cell->flags = 0;

            		nbor = cell->nbor;
            		if (x > 0) 
	        	{/* west */
				*(nbor++) = cell - 1;
      			}
      			if (x < dimx - 1)
	  		{/* east */
				*(nbor++) = cell + 1;
      			}
      			if (y > 0)
	  		{/* south */
				*(nbor++) = cell - dimx;
      			}
      			if (y < dimy - 1)
	  		{/* north */
				*(nbor++) = cell + dimx;
      			}
      			*nbor = 0;
      
      			nbor = cell->prop;
      			if (x > 0) 
	  		{
				if (y > 0) 
				{/* south-west */
	  				*(nbor++) = cell - 1;
	  				*(nbor++) = cell - dimx;
				}
				if (y < dimy - 1)
				{/* north-west */
	  				*(nbor++) = cell - 1;
	  				*(nbor++) = cell + dimx;
				}
      			}
      			if (x < dimx - 1)
	  		{
				if (y > 0)
				{/* south-east */
					*(nbor++) = cell + 1;
	  				*(nbor++) = cell - dimx;
				}
				if (y < dimy - 1)
				{/* north-east */
	  				*(nbor++) = cell + 1;
	 				*(nbor++) = cell + dimx;
				}
      			}
      			*nbor = 0;
    		}
     	}
}

void EstarROS::Estar_set_speed(estar_t *estar, size_t ix, size_t iy, double speed)
{
	
	//cout << " Inside set speed \n"; 
	double cost;
  	estar_cell_t * cell;
  	estar_cell_t ** nbor;
  	cell = estar_grid_at (&estar->grid, ix, iy);
	
	// XXXX I'm undecided yet whether this check here makes the most
   	// sense. The other option is to make sure that the caller doesn't
        // place obstacles into a goal cell. The latter somehow makes more
 	// sense to me at the moment, so in gestar.c there is code to filter
  	// goal cells from the obstacle setting routines.
  	////  if (cell->flags & ESTAR_FLAG_GOAL) {
  	////    return;
  	////  }
  
  	if (speed <= 0.0) 
  	{
   		cost = INFINITY;
  	}
  	else 
	{
    		cost = 1.0 / speed;
  	}
  	if (cost == cell->cost) 
	{
    		return;
  	}
  
  	cell->cost = cost;
  	if (speed <= 0.0) 
	{
    		cell->phi = INFINITY;
    		cell->rhs = INFINITY;
    		cell->flags |= ESTAR_FLAG_OBSTACLE;
  	}
  	else 
	{
    		cell->flags &= ~ESTAR_FLAG_OBSTACLE;
  	}
  
  	Estar_update(estar, cell);
  	for (nbor = cell->nbor; *nbor != 0; ++nbor) 
  	{
    		Estar_update (estar, *nbor);
  	}
}

void EstarROS::Estar_update(estar_t * estar, estar_cell_t * cell)
{
	// XXXX check whether obstacles actually can end up being
     	//updated. Possibly due to effects of estar_set_speed? 
  	if (cell->flags & ESTAR_FLAG_OBSTACLE) 
	{
    		Estar_pqueue_remove_or_ignore (&estar->pq, cell);
    		return;
  	}
  
  	// Make sure that goal cells remain at their rhs, which is supposed
     	//to be fixed and only serve as source for propagation, never as
     	//sink. 
  	if ( !(cell->flags & ESTAR_FLAG_GOAL)) 
	{
    		calc_rhs (cell, Estar_pqueue_topkey (&estar->pq));
  	}
  
  	if (cell->phi != cell->rhs) 
	{
    		Estar_pqueue_insert_or_update (&estar->pq, cell);
  	}
  	else
	{
    		Estar_pqueue_remove_or_ignore (&estar->pq, cell);
 	}
}

void EstarROS::Estar_pqueue_insert_or_update (estar_pqueue_t * pq, estar_cell_t * cell)
{
  	size_t len;
  	estar_cell_t ** heap;
  	//printf("cell->pqi = %zu\n",cell->pqi);
  	if (0 != cell->pqi) 
  	{
    	cell->key = CALC_KEY(cell);
    	// could probably make it more efficient by only bubbling down when
    	// the bubble up did not change cell->pqi
    	bubble_up (pq->heap, cell->pqi);
    	bubble_down (pq->heap, pq->len, cell->pqi);
    	return;
  	}
  
 	// grow heap, realloc if necessary
  	len = pq->len + 1;
  	if (len <= pq->cap) 
  	{
		//printf("pq->heap = %d\n",pq->heap);
    		heap = pq->heap;
  	}
  	else 
  	{
    	size_t cap;
    	cap = 2 * pq->cap;
    	heap =(estar_cell_t**) realloc (pq->heap, sizeof(estar_cell_t**) * (cap+1));
    	if (NULL == heap) 
	{
      		//errx (EXIT_FAILURE, __FILE__": %s: realloc", __func__);
    	}
    	pq->heap = heap;
    	pq->cap = cap;
  	}
  	pq->len = len;
  
 	// append cell to heap and bubble up
  	//printf("cell->rhs = %f,cell->phi = %f\n",cell->rhs,cell->phi);
  	cell->key = CALC_KEY(cell);
  	heap[len] = cell;
  	cell->pqi = len;		// initialize pqi 
  	bubble_up (heap, len);
	//printf("pqueue updated.\n");
}


void EstarROS::Estar_pqueue_remove_or_ignore (estar_pqueue_t * pq, estar_cell_t * cell)
{
	if (0 == cell->pqi) 
  	{
    		// This could be done by the caller for efficiency, but it is much
    		// more convenient to do it here.
   		 return;
  	}
  
  	pq->heap[cell->pqi] = pq->heap[pq->len];
  	pq->heap[cell->pqi]->pqi = cell->pqi; // keep pqi consistent! 
  	--pq->len;
  	bubble_down (pq->heap, pq->len, cell->pqi);
  	cell->pqi = 0;		// mark cell as not on queue 
}


void EstarROS::bubble_up (estar_cell_t ** heap, size_t index)
{
	size_t parent;
  	parent = index / 2;
  	//printf("parent = %zu\n",parent);
  	//printf("heap[index]->key = %zu\n",heap[index]->key);
  	//printf("heap[parent]->key = %zu\n",heap[parent]->key);	
  	while ((parent > 0) && (heap[index]->key < heap[parent]->key)) 
  	{
		//printf("into while of buble up\n");
    	swap (&heap[index], &heap[parent]);
    	index = parent;
    	parent = index / 2;
  	}
}

void EstarROS::bubble_down (estar_cell_t ** heap, size_t len, size_t index)
{
	size_t child, target;
  
  	target = index;
  	while (1)
	{
    	child = 2 * index;
    	if (child <= len && heap[child]->key < heap[target]->key)
	{
      		target = child;
    	}
    	++child;
    	if (child <= len && heap[child]->key < heap[target]->key)
	{
      		target = child;
    	}
    	if (index == target)
	{
     		 break;
    	}
    	swap (&heap[target], &heap[index]);
    	index = target;
  }
}

double EstarROS::Estar_pqueue_topkey (estar_pqueue_t * pq)
{
	if (pq->len > 0) 
	{
    		return pq->heap[1]->key;
  	}
  	return INFINITY;
}

void EstarROS::calc_rhs (estar_cell_t * cell, double phimax)
{
	estar_cell_t ** prop;
  	estar_cell_t * primary;
  	estar_cell_t * secondary;
  	double rr;
  
  	cell->rhs = INFINITY;
  	prop = cell->prop;
  	while (NULL != *prop) 
	{
    
    		primary = *(prop++);
    		if (primary->rhs <= (*prop)->rhs)  
		{
      			secondary = *(prop++);
    		}
    		else 
		{
      			secondary = primary;
      			primary = *(prop++);
    		}
    
    		// do not propagate from obstacles, queued cells, cells above the
    		// wavefront, or cells at infinity
    		if (primary->flags & ESTAR_FLAG_OBSTACLE
			|| primary->pqi != 0
			|| primary->phi > phimax
			|| isinf(primary->phi)) 
		{
     			 continue;
    		}
    
    		// the same goes from the secondary, but if that fails at least we
    		// can fall back to the non-interpolated update equation.
    		if (secondary->flags & ESTAR_FLAG_OBSTACLE
			|| secondary->pqi != 0
			|| secondary->phi > phimax
			|| isinf(secondary->phi))
		{
     		 	rr = primary->rhs + cell->cost;
    		}
    		else 
		{
      			rr = interpolate (cell->cost, primary->phi, secondary->phi);
    		}
    
    		if (rr < cell->rhs) 
		{
      			cell->rhs = rr;
    		}
  	}
  
  	if (isinf (cell->rhs)) 
	{
	    	// None of the above worked, we're probably done... but I have
    		// lingering doubts about about the effects of in-place primary /
    		// secondary sorting above, it could be imagined to create
    		// situations where we overlook something. So, just to be on the
    		// safe side, let's retry all non-interpolated options.
    		for (prop = cell->nbor; *prop != 0; ++prop) 
		{
      			rr = (*prop)->phi;
      			if (rr < cell->rhs)
			{
				cell->rhs = rr;
      			}
    		}
    		cell->rhs += cell->cost;
  	}
}

void EstarROS::swap (estar_cell_t ** aa, estar_cell_t ** bb)
{
	  size_t ti;
  	  estar_cell_t *tc;
  	  ti = (*aa)->pqi;
  	  (*aa)->pqi = (*bb)->pqi;
      	  (*bb)->pqi = ti;
      	  tc = (*aa);
          (*aa) = (*bb);
          (*bb) = tc;
}


void EstarROS::Estar_propagate (estar_t * estar)
{
	estar_cell_t * cell;
  	estar_cell_t ** nbor;
  
  	cell = Estar_pqueue_extract (&estar->pq);
  	if (NULL == cell) 
	{
    		return;
  	}
  
  	// The chunk below could be placed into a function called expand,
  	// but it is not needed anywhere else.
  
  	if (cell->phi > cell->rhs) 
	{
    		cell->phi = cell->rhs;
    		for(nbor = cell->nbor; *nbor != 0; ++nbor)
		{
      			Estar_update (estar, *nbor);
    		}
  	}
  	else 
	{
    		cell->phi = INFINITY;
    		for (nbor = cell->nbor; *nbor != 0; ++nbor) 
		{
      			Estar_update (estar, *nbor);
    		}
    		Estar_update (estar, cell);
  	}
}


int  EstarROS::Estar_check (estar_t * estar, char const * pfx)
{
	int status = 0;
  	size_t ii, jj, kk;
  
  	for (ii = 0; ii < estar->grid.dimx; ++ii) 
	{
	    for (jj = 0; jj < estar->grid.dimy; ++jj) 
	    {
 	    	estar_cell_t * cell;
      		cell = estar_grid_at (&estar->grid, ii, jj);
      
      		if (cell->rhs == cell->phi) 
		{
			// consistent
			if (0 != cell->pqi) 
			{
	  			printf ("%sconsistent cell should not be on queue\n", pfx);
	  			status |= 1;
			}
      		}
      		else 
		{
			// inconsistent
			if (0 == cell->pqi) 
			{
	  			printf ("%sinconsistent cell should be on queue\n", pfx);
	  			status |= 2;
			}
      		}
      
      		if (0 == cell->pqi) 
		{
			// not on queue
			for (kk = 1; kk <= estar->pq.len; ++kk) 
			{
	  			if (cell == estar->pq.heap[kk]) 
				{
	    				printf ("%scell with pqi == 0 should not be on queue\n", pfx);
	    				status |= 4;
	    				break;
	  			}
			}
      		}
      		else 
		{
			// on queue
			for (kk = 1; kk <= estar->pq.len; ++kk) 
			{
	  			if (cell == estar->pq.heap[kk]) 
				{
	   				 break;
	  			}
			}
			if (kk > estar->pq.len) 
			{
	 			printf ("%scell with pqi != 0 should be on queue\n", pfx);
	  			status |= 8;
			}
      		}
	    }
	}
  
  	for (ii = 1; ii <= estar->pq.len; ++ii) 
	{
    		if (estar->pq.heap[ii]->pqi != ii) 
		{
      			printf ("%sinconsistent pqi\n", pfx);
      			Estar_dump_queue (estar, pfx);
      			status |= 16;
      			break;
    		}
  	} 
  	return status;
}

void EstarROS::Estar_dump_queue (estar_t * estar, char const * pfx)
{
	size_t ii;
  	for (ii = 1; ii <= estar->pq.len; ++ii) 
  	{
    		printf ("%s[%zu %zu]  pqi:  %zu  key: %g  phi: %g  rhs: %g\n",
	    			pfx,
	    			(estar->pq.heap[ii] - estar->grid.cell) % estar->grid.dimx,
	    			(estar->pq.heap[ii] - estar->grid.cell) / estar->grid.dimx,
	    			estar->pq.heap[ii]->pqi, estar->pq.heap[ii]->key,
	    			estar->pq.heap[ii]->phi, estar->pq.heap[ii]->rhs);
  	}
}


estar_cell_t* EstarROS::Estar_pqueue_extract (estar_pqueue_t * pq)
{
	estar_cell_t * cell;
  
  	if (0 == pq->len) 
	{
    		return NULL;
  	}
  
 	cell = pq->heap[1];
  	cell->pqi = 0;		/* mark cell as not on queue */
  
  	if (1 == pq->len) 
	{
    		pq->len = 0;
    		return cell;
  	}
  
  	pq->heap[1] = pq->heap[pq->len];
  	pq->heap[1]->pqi = 1;		/* keep pqi consistent */
  	--pq->len;
  	// here would be a good place to shrink the heap
  
  	bubble_down (pq->heap, pq->len, 1);
  
  	return cell;
}

int EstarROS::Estar_cell_calc_gradient (estar_cell_t * cell, double * gx, double * gy)
{
	estar_cell_t ** nn;
  	estar_cell_t * n1;
  	estar_cell_t * n2;
  	int direction;
  
  	n1 = NULL;
  	for (nn = cell->nbor; *nn != NULL; ++nn) 
  	{
    		if (isfinite ((*nn)->rhs)
			&& (*nn)->rhs < cell->rhs
			&& (n1 == NULL || (*nn)->rhs < n1->rhs)) 
		{
      			n1 = *nn;
    		}
  	}
  	if (NULL == n1) 
	{
    		return 0;
  	}
  
  	direction = n1 - cell;
  	// +1 means right
  	// -1 means left
  	// +dimx means up (grid is arranged like pixels on a screen)
  	// -dimx means down
  
  	n2 = NULL;
  	for (nn = cell->nbor; *nn != NULL; ++nn) 
	{
    		if (isfinite ((*nn)->rhs)
			&& (*nn) != n1
			&& direction != cell - *nn /* check it is not opposite n1 */
			&& (n2 == NULL || (*nn)->rhs < n2->rhs)) 
		{
		      n2 = *nn;
    		}
  	}

 	if (NULL == n2) 
	{
    		if (direction == -1) 
		{
      			*gx = n1->rhs - cell->rhs; /* some negative value */
      			*gy = 0.0;
    		}
    		else if (direction == 1) 
		{
      			*gx = cell->rhs - n1->rhs; /* some positive value */
      			*gy = 0.0;
    		}
    		else if (direction < 0) 
		{
      			*gx = 0.0;
      			*gy = n1->rhs - cell->rhs; /* some negative value */
    		}
    		else 
		{
      			*gx = 0.0;
     		 	*gy = cell->rhs - n1->rhs; /* some positive value */
    		}
    		return 1;
  	}
  
  	if (direction == -1) 
	{
    		*gx = n1->rhs - cell->rhs;
    		if (cell - n2 > 0) 
		{
      			*gy = n2->rhs - cell->rhs;
    		}
    		else 
		{
      			*gy = cell->rhs - n2->rhs;
    		}	
  	}
  	else if (direction == 1) 
	{
    		*gx = cell->rhs - n1->rhs;
    		if (cell - n2 > 0) 
		{
      			*gy = n2->rhs - cell->rhs;
    		}
    		else 
		{
      			*gy = cell->rhs - n2->rhs;
    		}
  	}
  	else if (direction < 0) 
	{
    		if (cell - n2 > 0) 
		{
      			*gx = n2->rhs - cell->rhs;
    		}
    		else 
		{
      			*gx = cell->rhs - n2->rhs;
    		}
    		*gy = n1->rhs - cell->rhs;
  	}
  	else 
	{
    		if (cell - n2 > 0) 
		{
      			*gx = n2->rhs - cell->rhs;
    		}
    		else 
		{
      			*gx = cell->rhs - n2->rhs;
    		}
    		*gy = cell->rhs - n1->rhs;
  	}
  
  	return 2;
}

double EstarROS::interpolate(double cost, double primary, double secondary)
{
  double tmp;

  if (cost <= secondary - primary)
      return primary + cost;


  // pow(cost,2) could be cached inside estar_set_speed. And so could
  // the other squared terms. That might speed things up, but it would
  // certainly make hearier caching code.

  tmp = primary + secondary;
  return (tmp + sqrt(pow(tmp, 2.0)
		      - 2.0 * (pow(primary, 2.0)
			      + pow(secondary, 2.0)
			      - pow(cost, 2.0)))) / 2.0;
}

};
