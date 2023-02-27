#include<multibot_layers/path_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(multibot_layer_namespace::PathLayer, costmap_2d::Layer)

	using costmap_2d::LETHAL_OBSTACLE;

	namespace multibot_layer_namespace
{
	PathLayer::PathLayer() {}

	void PathLayer::onInitialize()
	{
		ros::NodeHandle nh("~/" + name_);
		current_ = true;

		dsrv_ = new dynamic_reconfigure::Server<multibot_layers::PathPluginConfig>(nh);
		dynamic_reconfigure::Server<multibot_layers::PathPluginConfig>::CallbackType cb = boost::bind(
				&PathLayer::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);
	}


	void PathLayer::reconfigureCB(multibot_layers::PathPluginConfig &config, uint32_t level)
	{
		total_vehicle_ = config.total_vehicle;
		index_ = config.index;
		enabled_ = config.enabled;
		prohibition_length_ = config.prohibition_length;
		path_vector_ = config.path_vector;
	}

	void PathLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
			double* min_y, double* max_x, double* max_y)
	{
		if (!enabled_)
			return;

		robot_yaw_ = robot_yaw;
		nh.setParam("pose_x", robot_x);
		nh.setParam("pose_y", robot_y);

		// dummy search for five robots. TODO: set dynamic robot array
		for (int i = 1; i < total_vehicle_+1; i++) {
			std::string robot_pose = "/erp42_";
			std::string robot_x;
			std::string robot_y;
			std::string this_bot;
			// search for the full posename of the current robot. Eg, this_bot = "/erp42_<current_robot_number>/pose_x".
			nh.searchParam("pose_x", this_bot);  


			robot_pose.append(std::to_string(i));  // set robot_pose to "/nb2_<i=1~5>"
			robot_x = robot_y = robot_pose;  // give robot_x and robot_y the same prefix
			robot_x.append("/pose_x");  // set robot_x to "/nb2_<i>/pose_x"
			robot_y.append("/pose_y");  // set robot_y to "/nb2_<i>/pose_y"

			// Save poses of every robot, except itself, in mark[][] array
			if (nh.searchParam(robot_x, robot_x) && robot_x != this_bot) {
				nh.getParam(robot_x, mark[i][0]);
				nh.getParam(robot_y, mark[i][1]);
			} else {
				mark[i][0] = 100;  // Set an unaffected pose for unused robots
				mark[i][1] = 100;  // Set an unaffected pose for unused robots
			} 

			*min_x = std::min(*min_x, mark[i][0]);
			*min_y = std::min(*min_y, mark[i][1]);
			*max_x = std::max(*max_x, mark[i][0]);
			*max_y = std::max(*max_y, mark[i][1]);
		}
	}

	void PathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
			int max_j)
	{
		if (!enabled_)
			return;
		unsigned int mx;
		unsigned int my;
		for (int i = 1; i < total_vehicle_+1; i++) {
			//ROS_INFO("set cost in %f", mark[i][0]);
			if(master_grid.worldToMap(mark[i][0], mark[i][1], mx, my)){

				if(i == index_ && master_grid.worldToMap(mark[i][0], mark[i][1], mx, my)){
					for (int n = 0; n < prohibition_length_; n++) {
						master_grid.setCost(mx+n*cos(path_vector_), my+n*sin(path_vector_), LETHAL_OBSTACLE);
					}
				}
			}
		}
	} 

} // end namespace
