/**
 * \brief the definition of visualization class
 * \author Shifan Zhu (shifzhu@gmail.com)
 * \date Dev. 21, 2020
 */

#include "visualization/visualization.h"

namespace visualization
{

namespace NDT
{

NormalDistributionTransformVisualization::NormalDistributionTransformVisualization(ros::NodeHandle &nh)
{
    sub_ellipsoid_ = nh.subscribe("/ndt_view", 10, &NormalDistributionTransformVisualization::EllipsoidArrayCallback, this);
    ros::spin();
}

NormalDistributionTransformVisualization::~NormalDistributionTransformVisualization(){}

void NormalDistributionTransformVisualization::EllipsoidArrayCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    visualization_msgs::MarkerArray ellipsoid_array;
}



}

}
