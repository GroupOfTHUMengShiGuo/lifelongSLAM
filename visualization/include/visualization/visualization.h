/**
 * \brief the definition of visualization class
 * \author Shifan Zhu (shifzhu@gmail.com)
 * \date Dev. 21, 2020
 */

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <stdio.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <glog/logging.h>
#include "ndt_view_msgs/NDTView.h"
#include "ndt_view_msgs/NDTViewArray.h"

namespace visualization
{

namespace NDT
{

class NormalDistributionTransformVisualization
{
public:
    /**
     * \brief get ellipsoid array from topic.
     * param[in] 
     * param[out] ros MarkerArray message.
     */
    void SphereArrayCallback (const ndt_view_msgs::NDTViewArray& pc2);
    // void SphereArrayCallback (const sensor_msgs::PointCloud2ConstPtr& pc2);

    /**
     * \brief default constructor
     * param[in] ros node handle
     */
    NormalDistributionTransformVisualization(ros::NodeHandle &nh);

    /**
     * \brief default deconstructor
     * param[in] ros node handle
     */
    ~NormalDistributionTransformVisualization();

private:
    /**
     * \brief topic publisher and subscriber
     */
    ros::Subscriber sub_sphere_;
    ros::Publisher pub_sphere_;
};

}

}

#endif