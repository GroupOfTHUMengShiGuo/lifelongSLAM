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
    sub_sphere_ = nh.subscribe("/NDTView_msg", 10, &NormalDistributionTransformVisualization::SphereArrayCallback, this);
    pub_sphere_ = nh.advertise<visualization_msgs::MarkerArray>("/ndt_spheres", 5);
    ros::spin();
}

NormalDistributionTransformVisualization::~NormalDistributionTransformVisualization(){}

// void NormalDistributionTransformVisualization::SphereArrayCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
void NormalDistributionTransformVisualization::SphereArrayCallback(const ndt_view_msgs::NDTViewArray& msg) {
  int sphere_id = 0;
  visualization_msgs::MarkerArray sphere_array;

  Eigen::Matrix3d ndt_cov_mat, eigen_mat;
  Eigen::Vector3d eigen_val, eigen_vec, ndt_view_position;
  // ndt_cov_mat << 0.220307, 0.0716173, -0.0692341, 0.0716173, 0.174037, -0.0383424,
  //                -0.0692341, -0.0383424, 0.118082;
  for (int i = 0; i < msg.leaves.size(); ++i) {
    if (msg.leaves[i].covs.size() == 9) {
      for (int j = 0; j < msg.leaves[i].covs.size(); ++j) {
        ndt_cov_mat(j/3,j%2) = msg.leaves[i].covs[j];
      }
    } else {
      std::cerr << "ERROR: Wrong covirance dimention" << std::endl;
      continue;
    }
    if (msg.leaves[i].position.size() == 3) {
      for (int j = 0; j < msg.leaves[i].position.size(); ++j) {
        ndt_view_position[j] = msg.leaves[i].position[j];
      }
    } else {
      std::cerr << "ERROR: Wrong position dimention" << std::endl;
      continue;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(ndt_cov_mat);
    if(eigen_solver.info() == Eigen::Success) {
      eigen_val = eigen_solver.eigenvalues();
      // TODO bug
      // CHECK_GE(eigen_val[0], 0);
      // CHECK_GE(eigen_val[1], 0);
      // CHECK_GE(eigen_val[2], 0);
      Eigen::Matrix3d eigen_mat;
      eigen_mat.block(0,0,3,1) = eigen_solver.eigenvectors().col(2);
      eigen_mat.block(0,1,3,1) = eigen_solver.eigenvectors().col(1);
      eigen_mat.block(0,2,3,1) = eigen_solver.eigenvectors().col(0);
      Eigen::Quaterniond eigen_quat(eigen_mat);

      visualization_msgs::Marker sphere_temp;
      sphere_temp.header.stamp = ros::Time::now();
      sphere_temp.header.frame_id = "odom";
      sphere_temp.ns = "ndt_spheres";
      sphere_temp.type = visualization_msgs::Marker::SPHERE;
      sphere_temp.id = msg.leaves[i].id;
      sphere_temp.action = visualization_msgs::Marker::ADD;
      sphere_temp.pose.orientation.x = eigen_quat.x();
      sphere_temp.pose.orientation.y = eigen_quat.y();
      sphere_temp.pose.orientation.z = eigen_quat.z();
      sphere_temp.pose.orientation.w = eigen_quat.w();
      sphere_temp.scale.x = eigen_val[2];
      sphere_temp.scale.y = eigen_val[1];
      sphere_temp.scale.z = eigen_val[0];
      sphere_temp.color.r = 10;
      sphere_temp.color.g = 10;
      sphere_temp.color.b = 10;
      sphere_temp.color.a = 0.5;
      sphere_temp.pose.position.x = ndt_view_position[0];
      sphere_temp.pose.position.y = ndt_view_position[1];
      sphere_temp.pose.position.z = ndt_view_position[2];
      sphere_array.markers.push_back(sphere_temp);
    } else {
      std::cout << "Can't compute eigen values" << std::endl;
    }
  }

  pub_sphere_.publish(sphere_array);
  sphere_array.markers.clear();
}

}

}
