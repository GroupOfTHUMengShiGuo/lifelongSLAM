//
// Created by zxd on 2020/12/23.
//

#ifndef SRC_NDT_TO_VIEWMSG_H
#define SRC_NDT_TO_VIEWMSG_H
#include <Eigen/Core>
#include "ndt_view_msgs/NDTView.h"
#include "ndt_view_msgs/NDTViewArray.h"
namespace communication_middleware {
class NdtToViewmsg {
 public:
  NdtToViewmsg() {};

//  将体素的均值方差信息转换为可显示的rosmsg
//  参数1:输入，体素的方差
//  参数2:输入，体素的均值
//  参数3:输出，rosmsg
//  return 如果成功转换则return true
  bool VoxelGridCovarianceToViewmsg(const Eigen::Matrix3d& cov, const Eigen::Vector3d& mean, const uint64_t id, ndt_view_msgs::NDTView* msg);
};
}
#endif //SRC_NDT_TO_VIEWMSG_H
