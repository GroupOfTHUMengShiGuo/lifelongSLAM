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
    pub_ = nh.advertise<visualization_msgs::MarkerArray>("/ndt_spheres", 5);
    ros::spin();
}

NormalDistributionTransformVisualization::~NormalDistributionTransformVisualization(){}

void NormalDistributionTransformVisualization::SphereArrayCallback(const ndt_view_msgs::NDTViewArray& msg) {
  int sphere_id = 0;
  visualization_msgs::MarkerArray sphere_array;
  visualization_msgs::MarkerArray text_array;
  visualization_msgs::MarkerArray arrow_array;
  // Publish Cube list
  // TODO make the cube list visualization optional
  // TODO expand the display range automatically
  visualization_msgs::Marker cube_list;
  cube_list.header.stamp = ros::Time::now();
  cube_list.header.frame_id = msg.frame_id;
  cube_list.ns = "cubes";
  cube_list.type = visualization_msgs::Marker::CUBE_LIST;
  cube_list.id = -1;
  cube_list.action = visualization_msgs::Marker::ADD;
  cube_list.lifetime = ros::Duration();
  cube_list.scale.x = msg.resolution;
  cube_list.scale.y = msg.resolution;
  cube_list.scale.z = msg.resolution;
  cube_list.color.a = 0.3;
  cube_list.color.b = 0.5;
  cube_list.color.g = 0.5;

  Eigen::Matrix3d ndt_cov_mat, eigen_mat;
  Eigen::Vector3d eigen_val, eigen_vec, ndt_view_position;

  //变量表示这一帧内所有椭圆主方向的平均特征值大小
  double the_mean_max_eigenval = 0;
  //变量表示所有成功分解的方差矩阵的数量
  int num_Eigen_Success = 0;
  for (int i = 0; i < msg.leaves.size(); ++i) {
    if (msg.leaves[i].covs.size() == 9) {
      for (int j = 0; j < msg.leaves[i].covs.size(); ++j) {
        ndt_cov_mat(j/3,j%3) = msg.leaves[i].covs[j];
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
      the_mean_max_eigenval = 1.0 * num_Eigen_Success / (num_Eigen_Success+1) * the_mean_max_eigenval +
                              1.0 / (num_Eigen_Success+1) * eigen_val[2];
      num_Eigen_Success++;
      // TODO bug
      // CHECK_GE(eigen_val[0], 0);
      // CHECK_GE(eigen_val[1], 0);
      // CHECK_GE(eigen_val[2], 0);
      eigen_mat.block(0,0,3,1) = eigen_solver.eigenvectors().col(0);
      eigen_mat.block(0,1,3,1) = eigen_solver.eigenvectors().col(1);
      eigen_mat.block(0,2,3,1) = eigen_solver.eigenvectors().col(2);

      if(abs(eigen_mat.determinant()+1) < 0.001) {
        eigen_mat.col(0) = - eigen_mat.col(0);
      }

      Eigen::Quaterniond eigen_quat(eigen_mat);
      eigen_quat.normalize();
      visualization_msgs::Marker sphere_temp;
      sphere_temp.header.stamp = ros::Time::now();
      sphere_temp.header.frame_id = msg.frame_id;
      sphere_temp.ns = "ndt_spheres";
      sphere_temp.type = visualization_msgs::Marker::SPHERE;
      sphere_temp.id = msg.leaves[i].id * 5;
      sphere_temp.action = visualization_msgs::Marker::ADD;
      sphere_temp.lifetime = ros::Duration();
      sphere_temp.pose.orientation.x = eigen_quat.x();
      sphere_temp.pose.orientation.y = eigen_quat.y();
      sphere_temp.pose.orientation.z = eigen_quat.z();
      sphere_temp.pose.orientation.w = eigen_quat.w();
      sphere_temp.scale.x = eigen_val[0];
      sphere_temp.scale.y = eigen_val[1];
      sphere_temp.scale.z = eigen_val[2];
      sphere_temp.color.r = 10;
      sphere_temp.color.g = 10;
      sphere_temp.color.b = 10;
      sphere_temp.color.a = 0.5;
      sphere_temp.pose.position.x = ndt_view_position[0];
      sphere_temp.pose.position.y = ndt_view_position[1];
      sphere_temp.pose.position.z = ndt_view_position[2];
      sphere_array.markers.push_back(sphere_temp);

      geometry_msgs::Point point_temp;
      point_temp.x = msg.leaves[i].cube_position[0];
      point_temp.y = msg.leaves[i].cube_position[1];
      point_temp.z = msg.leaves[i].cube_position[2];
      cube_list.points.push_back(point_temp);
      cube_list.colors.push_back(cube_list.color);

      visualization_msgs::Marker text_temp;
      text_temp.header.stamp = ros::Time::now();
      text_temp.header.frame_id = msg.frame_id;
      text_temp.ns = "ndt_spheres";
      text_temp.id = msg.leaves[i].id * 5 + 1;
      text_temp.action = visualization_msgs::Marker::ADD;
      text_temp.lifetime = ros::Duration();
      text_temp.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_temp.pose.position.x = point_temp.x;
      text_temp.pose.position.y = point_temp.y;
      text_temp.pose.position.z = point_temp.z;
      text_temp.scale.z = 0.1;
      text_temp.color.g = 1.0f;
      text_temp.color.a = 1;
      text_temp.text = std::to_string(msg.leaves[i].id);
      text_array.markers.push_back(text_temp);

      for (int t = 0; t < 3; t++) {
        visualization_msgs::Marker arrow_temp;
        arrow_temp.header.stamp = ros::Time::now();
        arrow_temp.header.frame_id = msg.frame_id;
        arrow_temp.ns = "ndt_spheres";
        arrow_temp.id = sphere_temp.id + 2 + t;
        arrow_temp.action = visualization_msgs::Marker::ADD;
        arrow_temp.lifetime = ros::Duration();
        arrow_temp.type = visualization_msgs::Marker::ARROW;
        arrow_temp.scale.x = 0.1;
        arrow_temp.scale.y = 0.1;
        arrow_temp.scale.z = 0.1;
        arrow_temp.color.r = 1.0;
        arrow_temp.color.a = 0.5;
        geometry_msgs::Point p1, p2;
        p1.x = sphere_temp.pose.position.x;
        p1.y = sphere_temp.pose.position.y;
        p1.z = sphere_temp.pose.position.z;
        p2.x = sphere_temp.pose.position.x + eigen_mat.col(t)[0];
        p2.y = sphere_temp.pose.position.y + eigen_mat.col(t)[1];
        p2.z = sphere_temp.pose.position.z + eigen_mat.col(t)[2];
        arrow_temp.points.push_back(p1) ;
        arrow_temp.points.push_back(p2) ;
        arrow_array.markers.push_back(arrow_temp);
      }
    } else {
      std::cout << "Can't compute eigen values" << std::endl;
    }
  }
  if (the_mean_max_eigenval != 0) {
    for (int i = 0; i < sphere_array.markers.size(); i++) {
      auto &sphere_temp = sphere_array.markers[i];
      sphere_temp.scale.x /= the_mean_max_eigenval;
      sphere_temp.scale.y /= the_mean_max_eigenval;
      sphere_temp.scale.z /= the_mean_max_eigenval;
      for (int t = 0; t < 3; t++) {
        auto &arrow_temp = arrow_array.markers[i * 3 + t];
        double delta_x, delta_y, delta_z;
        delta_x = arrow_temp.points[1].x - arrow_temp.points[0].x;
        delta_y = arrow_temp.points[1].y - arrow_temp.points[0].y;
        delta_z = arrow_temp.points[1].z - arrow_temp.points[0].z;
        switch (t) {
          case 0: {
            arrow_temp.points[1].x = sphere_temp.scale.x * delta_x / 2 + arrow_temp.points[0].x;
            arrow_temp.points[1].y = sphere_temp.scale.x * delta_y / 2 + arrow_temp.points[0].y;
            arrow_temp.points[1].z = sphere_temp.scale.x * delta_z / 2 + arrow_temp.points[0].z;
            break;
          }
          case 1: {
            arrow_temp.points[1].x = sphere_temp.scale.y * delta_x / 2 + arrow_temp.points[0].x;
            arrow_temp.points[1].y = sphere_temp.scale.y * delta_y / 2 + arrow_temp.points[0].y;
            arrow_temp.points[1].z = sphere_temp.scale.y * delta_z / 2 + arrow_temp.points[0].z;
            break;
          }
          case 2: {
            arrow_temp.points[1].x = sphere_temp.scale.z * delta_x / 2 + arrow_temp.points[0].x;
            arrow_temp.points[1].y = sphere_temp.scale.z * delta_y / 2 + arrow_temp.points[0].y;
            arrow_temp.points[1].z = sphere_temp.scale.z * delta_z / 2 + arrow_temp.points[0].z;
            break;
          }
          default: {
            assert(false);
          }
        }
      }
    }
  }

  // sphere_array.markers.push_back(cube_list);

  pub_.publish(sphere_array);
  // pub_.publish(text_array);
  // pub_.publish(arrow_array);
  sphere_array.markers.clear();
}

}

}
