#ifndef PATH_LAYER_H_
#define PATH_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <multibot_layers/PathPluginConfig.h>
#include <dynamic_reconfigure/server.h>
 
namespace multibot_layer_namespace
{

class PathLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  PathLayer();
 
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
 
private:
  void reconfigureCB(multibot_layers::PathPluginConfig &config, uint32_t level);

  
  double mark[10][2];
  int total_vehicle_;
  int index_;
  double robot_yaw_;
  // bool right_or_left_;
  double prohibition_length_;
  double path_vector_;
  ros::NodeHandle nh;
  dynamic_reconfigure::Server<multibot_layers::PathPluginConfig> *dsrv_;
};
}
#endif
