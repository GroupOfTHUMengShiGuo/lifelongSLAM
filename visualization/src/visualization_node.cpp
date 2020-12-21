/**
 * \brief the definition of visualization class
 * \author Shifan Zhu (shifzhu@gmail.com)
 * \date Dev. 21, 2020
 */

#include "visualization/visualization.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "visualization");
    ros::NodeHandle nh_visual;
    visualization::NDT::NormalDistributionTransformVisualization ndt_visual(nh_visual);
    return 0;
}