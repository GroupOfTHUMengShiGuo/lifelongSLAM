//
// Created by zxd on 2020/12/23.
//
#include "communication_middleware/ndt_to_viewmsg.h"
namespace communication_middleware {
 bool NdtToViewmsg::VoxelGridCovarianceToViewmsg(const Eigen::Matrix3d &cov, const Eigen::Vector3d &mean,
                                                 const uint64_t id, const Eigen::Array4d &cube_position,
                                                 ndt_view_msgs::NDTView *msg) {
   for (int i = 0; i < 3; ++i) {
     for (int j = 0; j < 3; ++j) {
       msg->covs.push_back(cov(i, j));
     }
   }
   for (int i = 0; i < 3; ++i) {
     msg->position.push_back(mean[i]);
     msg->cube_position.push_back(cube_position[i]);
   }
   msg->id = id;
   return true;
 }
}