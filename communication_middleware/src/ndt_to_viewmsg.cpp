//
// Created by zxd on 2020/12/23.
//
#include "communication_middleware/ndt_to_viewmsg.h"
namespace communication_middleware {
 bool NdtToViewmsg::VoxelGridCovarianceToViewmsg(const Eigen::Matrix3d &cov, const Eigen::Vector3d &mean,
                                                const uint64_t id, ndt_view_msgs::NDTView *msg) {
  // ndt_cov_mat << 0.220307, 0.0716173, -0.0692341, 0.0716173, 0.174037, -0.0383424,
  //                -0.0692341, -0.0383424, 0.118082;
   if (cov.size() == 9) {
     for (int i = 0; i < 3; ++i) {
       for (int j = 0; j < 3; ++j) {
         msg->covs.push_back(cov(i, j));
       }
     }
   } else {
     std::cerr << "ERROR: Wrong covirance dimention" << std::endl;
     return false;
   }
   if (mean.size() == 3) {
     for (int i = 0; i < 3; ++i) {
       msg->position.push_back(mean[i]);
     }
   } else {
     std::cerr << "ERROR: Wrong position dimention" << std::endl;
     return false;
   }
   msg->id = id;
   return true;
 }
}