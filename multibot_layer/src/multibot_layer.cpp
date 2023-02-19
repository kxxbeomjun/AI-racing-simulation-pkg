#include<multibot_layers/multibot_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(multibot_layer_namespace::MultibotLayer, costmap_2d::Layer)

	using costmap_2d::LETHAL_OBSTACLE;

	namespace multibot_layer_namespace
{
	MultibotLayer::MultibotLayer() : resolution_(0)
	, inflation_radius_(0)
	, inscribed_radius_(0)
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
	, robot_yaw_(0.0)
	{
		inflation_access_ = new boost::recursive_mutex();
	}

	void MultibotLayer::onInitialize()
	{
		ros::NodeHandle nh("~/" + name_);
		current_ = true;

		dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb =
        [this](auto& config, auto level){ reconfigureCB(config, level); };

		if (dsrv_ != NULL){
		dsrv_->clearCallback();
		dsrv_->setCallback(cb);
		}
		else
		{
		dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
		dsrv_->setCallback(cb);
		}

		vehicle_num_ = 0;
		nh.getParam("total_vehicle", total_vehicle_);
		nh.getParam("inflation_angle", inflation_angle_);
		ROS_INFO("Total vehicle: %d", total_vehicle_);
		
		current_ = true;
		if (seen_)
		delete[] seen_;
		seen_ = NULL;
		seen_size_ = 0;
		need_reinflation_ = false;
		ROS_INFO("on Initialize multibot layer");
		matchSize();
		ROS_INFO("Initialize multibot layer completed");
		}


	void MultibotLayer::matchSize()
	{
		ROS_INFO("matchSize");
		boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
		costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
		resolution_ = costmap->getResolution();
		cell_inflation_radius_ = cellDistance(inflation_radius_);
		computeCaches();

		unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
		if (seen_)
			delete[] seen_;
		seen_size_ = size_x * size_y;
		seen_ = new bool[seen_size_];
	}

	void MultibotLayer::computeCaches()
	{
		ROS_INFO("computeCaches");
		if (cell_inflation_radius_ == 0)
			return;

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
				cached_distances_[i][j] = hypot(i, j);
			}
			}

			cached_cell_inflation_radius_ = cell_inflation_radius_;
		}

		for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
		{
			for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
			{
			cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
			}
		}
		ROS_INFO("computeCaches done");
	}

	void MultibotLayer::deleteKernels()
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


	void MultibotLayer::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level)
	{
		enabled_ = config.enabled;
		setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

		if (enabled_ != config.enabled || inflate_unknown_ != config.inflate_unknown) {
			enabled_ = config.enabled;
			inflate_unknown_ = config.inflate_unknown;
			need_reinflation_ = true;
		}
	}

	void MultibotLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y)
	{
		if (!enabled_)
			return;
		
		robot_yaw_ = robot_yaw;
		nh.setParam("pose_x", robot_x);
		nh.setParam("pose_y", robot_y);

		for (int i = 1; i < total_vehicle_+1; i++) {
			std::string robot_pose = "/erp42_";
			std::string robot_x;
			std::string robot_y;
			std::string this_bot;
			// search for the full posename of the current robot. Eg, this_bot = "/erp42_<current_robot_number>/pose_x".
			nh.searchParam("pose_x", this_bot);  

			
			robot_pose.append(std::to_string(i));  // set robot_pose to "/erp42_<i=1~>"
			robot_x = robot_y = robot_pose;  // give robot_x and robot_y the same prefix
			robot_x.append("/pose_x");  // set robot_x to "/erp42_<i>/pose_x"
			robot_y.append("/pose_y");  // set robot_y to "/erp42_<i>/pose_y"
			// ROS_INFO("Seach param: %s", robot_x.c_str());
			// ROS_INFO("this_bot: %s", this_bot.c_str());
			// Save poses of every robot, except itself, in mark[][] array
			if (nh.searchParam(robot_x, robot_x) && robot_x != this_bot) {
				nh.getParam(robot_x, mark[i][0]);
				nh.getParam(robot_y, mark[i][1]);
			} else {
				vehicle_num_ = i;
				mark[i][0] = 100;  // Set an unaffected pose for unused robots
				mark[i][1] = 100;  // Set an unaffected pose for unused robots
			} 

			*min_x = std::min(*min_x, mark[i][0]);
			*min_y = std::min(*min_y, mark[i][1]);
			*max_x = std::max(*max_x, mark[i][0]);
			*max_y = std::max(*max_y, mark[i][1]);
		}
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

	void MultibotLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
			int max_j)
	{
		boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

		if (!enabled_)
			return;
		unsigned int mx;
		unsigned int my;

		unsigned char* master_array = master_grid.getCharMap();
		unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

		if (seen_ == NULL) {
			ROS_WARN("MultibotLayer::updateCosts(): seen_ array is NULL");
			seen_size_ = size_x * size_y;
			seen_ = new bool[seen_size_];
		}
		else if (seen_size_ != size_x * size_y)
		{
			ROS_WARN("MultibotLayer::updateCosts(): seen_ array size is wrong");
			delete[] seen_;
			seen_size_ = size_x * size_y;
			seen_ = new bool[seen_size_];
		}
		memset(seen_, false, size_x * size_y * sizeof(bool));

		min_i -= cell_inflation_radius_;
		min_j -= cell_inflation_radius_;
		max_i += cell_inflation_radius_;
		max_j += cell_inflation_radius_;

		min_i = std::max(0, min_i);
		min_j = std::max(0, min_j);
		max_i = std::min(int(size_x), max_i);
		max_j = std::min(int(size_y), max_j);

		std::vector<CellData>& obs_bin = inflation_cells_[0.0];
	
		for (int i = 1; i < total_vehicle_+1; i++) {
			//ROS_INFO("set cost in %f", mark[i][0]);
			if(master_grid.worldToMap(mark[i][0], mark[i][1], mx, my)){
				int index = master_grid.getIndex(mx, my);;
				obs_bin.push_back(CellData(index, mx, my, mx, my));
			}
		}

		std::map<double, std::vector<CellData> >::iterator bin;
		for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
		{
			for (int i = 0; i < bin->second.size(); ++i)
			{
				// process all cells at distance dist_bin.first
				const CellData& cell = bin->second[i];

				unsigned int index = cell.index_;

				// ignore if already visited
				if (seen_[index])
				{
					continue;
				}

				seen_[index] = true;

				unsigned int mx = cell.x_;
				unsigned int my = cell.y_;
				unsigned int sx = cell.src_x_;
				unsigned int sy = cell.src_y_;

				// assign the cost associated with the distance from an obstacle to the cell
				unsigned char cost = costLookup(mx, my, sx, sy);
				unsigned char old_cost = master_array[index];
				if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
					master_array[index] = cost;
				else
					master_array[index] = std::max(old_cost, cost);

				// attempt to put the neighbors of the current cell onto the inflation list
				if (mx > 0)
					enqueue(index - 1, mx - 1, my, sx, sy);
				if (my > 0)
					enqueue(index - size_x, mx, my - 1, sx, sy);
				if (mx < size_x - 1)
					enqueue(index + 1, mx + 1, my, sx, sy);
				if (my < size_y - 1)
					enqueue(index + size_x, mx, my + 1, sx, sy);
				}
			}

		inflation_cells_.clear();
		

		
	}

	void MultibotLayer::onFootprintChanged()
	{
		//inscribed_radius_ = layered_costmap_->getInscribedRadius();
		inscribed_radius_ = 0.75;
		cell_inflation_radius_ = cellDistance(inflation_radius_);
		computeCaches();
		need_reinflation_ = true;

		ROS_DEBUG("MultibotLayer::onFootprintChanged(): num footprint points: %lu,"
					" inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
					layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
	} 

	void MultibotLayer::setInflationParameters(double inflation_radius, double cost_scaling_factor)
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

	inline void MultibotLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y)
	{
		int mx_int = static_cast<int>(mx);
		int my_int = static_cast<int>(my);
		int src_x_int = static_cast<int>(src_x);
		int src_y_int = static_cast<int>(src_y);

		if (!seen_[index])
		{
			// we compute our distance table one cell further than the inflation radius dictates so we can make the check below
			double distance = distanceLookup(mx, my, src_x, src_y);

			// we only want to put the cell in the list if it is within the inflation radius of the obstacle point
			if (distance > cell_inflation_radius_)
			return;

			if (std::abs(mx_int-src_x_int)+std::abs(my_int-src_y_int)>1 && std::abs(atan2(src_y_int-my_int, src_x_int-mx_int)-robot_yaw_)>inflation_angle_/2)
			return;

			// push the cell data onto the inflation list and mark
			inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
		}
	}

} // end namespace
