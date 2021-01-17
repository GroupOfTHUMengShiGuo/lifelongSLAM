//
// Created by zxd on 20-10-16.
//
#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>

#include <ndt_update.h>
//#include <ndt_update.hpp>
#include <voxel_grid_covariance_update.h>
#include <voxel_grid_covariance_update.hpp>

#include <communication_middleware/ndt_to_viewmsg.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "ros/ros.h"
using namespace std ::literals ::chrono_literals;
// Initializing Normal Distributions Transform (NDT).
static pcl_update::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>
    ndt;
static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_ori;

//原始ndt的结果点云
static pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
// updatendt的结果点云
static pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_update(
    new pcl::PointCloud<pcl::PointXYZ>);
//原始ndt的结果点云发送到rviz上
static sensor_msgs::PointCloud2 ori_output;
// updatendt的结果点云发送到rviz上
static sensor_msgs::PointCloud2 update_output;
// update的单帧结果点云发送到rviz上
static sensor_msgs::PointCloud2 update_single_output;
//显示ndt地图的点云
static sensor_msgs::PointCloud2 ndt_viz;
//用来显示的Oc点云
static pcl::PointCloud<pcl::PointXYZI>::Ptr Oc_pointcloud(
    new pcl::PointCloud<pcl::PointXYZI>);
//用来单帧显示的Oc点云
static pcl::PointCloud<pcl::PointXYZI>::Ptr single_oc_pointcloud(
    new pcl::PointCloud<pcl::PointXYZI>);
//申明发布器
ros::Publisher pub;
//申明ndt_msgs发布器
ros::Publisher pub_ndt_msgs;
static size_t counter = 0;
void SubscribePointCloud(
    const sensor_msgs::PointCloud2ConstPtr &lidar_message) {
  pcl::PointRepresentation<pcl::PointXYZ>::ConstPtr point_representation(
      new pcl::DefaultPointRepresentation<pcl::PointXYZ>);
  //记录分数之差
  static double ori_sum = 0, diff_sum = 0;
  //记录前一帧位姿
  static Eigen::Matrix4f T_last = ndt.getFinalTransformation();
  static Eigen::Matrix4f T_last_ori = ndt_ori.getFinalTransformation();
  //记录当前位姿
  static Eigen::Matrix4f T_now = ndt.getFinalTransformation();
  static Eigen::Matrix4f T_now_ori = ndt_ori.getFinalTransformation();

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_temp(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidar_message, *input_cloud_temp);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ori(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &input_point : *input_cloud_temp) {
    if (point_representation->isValid(input_point) &&
        input_point.z >= -2.6
        ) {
      input_cloud_ori->push_back(input_point);
    }
  }
  counter++;
  // 进行统计滤波去除离群点
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(input_cloud_ori);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1);
  sor.setNegative(false);
  sor.filter(*input_cloud);
  // Loading first scan of room.
  if (input_cloud->empty()) {
    // /media/zxd/60787A51787A25C6/pcl_ndt/PointCloud/bin2pcd/velodyne/pcd/1025/
    PCL_ERROR("Couldn't read file in package \n");
    return;
  }
  std::cout << "Loaded " << input_cloud->size()
            << " data points from " + std::to_string(counter) << std::endl;

  if (!(counter - 1)) {
    *target_cloud = *input_cloud;
    *target_cloud_update = *input_cloud;
    // 设置初始要建立ndt地图的点云.
    ndt.updateInputTarget(target_cloud_update);
//    ndt_ori.setInputTarget(target_cloud);
    return;
  }
  // Filtering input scan to roughly 10% of original size to increase speed of
  // registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  //降采样之后的update点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_update(
      new pcl::PointCloud<pcl::PointXYZ>);
  //降采样之后的ori点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ori(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud(input_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
  //将要加入结果点云的数据进行降采样
  approximate_voxel_filter.setLeafSize(0.01, 0.01, 0.01);
  approximate_voxel_filter.filter(*filtered_cloud_update);
//  approximate_voxel_filter.filter(*filtered_cloud_ori);
  std::cout << "Filtered cloud contains " << filtered_cloud_update->size()
            << " data points from room_scan2.pcd" << std::endl;
  // Setting point cloud to be aligned.
  ndt.setInputSource(filtered_cloud);
//  ndt_ori.setInputSource(filtered_cloud);
  //按照匀速模型设置初始位姿
  Eigen::Matrix4f init_guess = T_now * T_last.inverse() * T_now;
//  Eigen::Matrix4f init_guess_ori = T_now_ori * T_last_ori.inverse() * T_now_ori;

  // Calculating required rigid transform to align the input cloud to the target
  // cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ori(
//      new pcl::PointCloud<pcl::PointXYZ>);

  //开始匹配
  start = clock();  //程序开始计时
  ndt.align(*output_cloud, init_guess);
  end = clock();  //程序结束计时
  double endtime = (double)(end - start) / CLOCKS_PER_SEC;
  ofstream write;
  write.open("logfiles/update_align_time.txt",
             ios::app);  //用ios::app不会覆盖文件内容
  write << endtime * 1000 << endl;
  write.close();
  // std::cout<<"update_align time:"<<endtime*1000<<"ms"<<std::endl;
  // //ms为单位

  start = clock();  //程序开始计时
//  ndt_ori.align(*output_cloud_ori, init_guess_ori);
  end = clock();  //程序结束计时
  endtime = (double)(end - start) / CLOCKS_PER_SEC;
  write.open("logfiles/ori_align_time.txt",
             ios::app);  //用ios::app不会覆盖文件内容
  write << endtime * 1000 << endl;
  write.close();
  // std::cout<<"ori_align time:"<<endtime*1000<<"ms"<<std::endl;
  // //ms为单位

//  std::cout << "Normal Distributions Transform has converged:"
//            << ndt.hasConverged() << " update_score: " << ndt.getFitnessScore()
//            << std::endl;
//  std::cout << "Normal Distributions Transform has converged:"
//            << ndt_ori.hasConverged()
//            << " ori_score: " << ndt_ori.getFitnessScore() << std::endl;
//  ori_sum = ndt_ori.getFitnessScore();
//  diff_sum = diff_sum + (ndt.getFitnessScore() - ndt_ori.getFitnessScore()) /
//                            ori_sum * 100;
//  cout << "diff = " << diff_sum << "%" << endl;
  //更新上一帧和本帧位姿
  T_last = T_now;
  T_last_ori = T_now_ori;
  // TODO
  // 将原始位姿作为新点云的转换，目的是为了观察位姿相同的情况下的均值方差，之后需要改回来
  T_now = ndt.getFinalTransformation();
  T_now_ori = ndt_ori.getFinalTransformation();
  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud(*filtered_cloud_update, *output_cloud,
                           T_now);
//  pcl::transformPointCloud(*filtered_cloud_ori, *output_cloud_ori,
//                           T_now_ori);
  // 更新ndt地图
  start = clock();  //程序开始计时
  ndt.updateInputTarget(output_cloud);
  end = clock();  //程序结束计时
  endtime = (double)(end - start) / CLOCKS_PER_SEC;
  write.open("logfiles/update_mapping_time.txt",
             ios::app);  //用ios::app不会覆盖文件内容
  write << endtime * 1000 << endl;
  write.close();
  // std::cout<<"update_mapping time:"<<endtime*1000<<"ms"<<std::endl;
  // //ms为单位

  // Saving result cloud.
  //  pcl::io::savePCDFileASCII ("output.pcd", *output_cloud);

  //将转换后的地图加入到
//  *target_cloud = *target_cloud + *output_cloud_ori;
  *target_cloud_update = *target_cloud_update + *output_cloud;

  start = clock();  //程序开始计时
//  ndt_ori.setInputTarget(target_cloud);
  end = clock();  //程序结束计时
  endtime = (double)(end - start) / CLOCKS_PER_SEC;
  write.open("logfiles/ori_mapping_time.txt",
             ios::app);  //用ios::app不会覆盖文件内容
  write << endtime * 1000 << endl;
  write.close();
  // std::cout<<"ori_mapping time:"<<endtime*1000<<"ms"<<std::endl;
  // //ms为单位

  //用点云表示ndt地图
  pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  ndt.getTarget_cells().getDisplayCloud(*ndt_cloud);

  // Saving ndt cloud.
  // pcl::io::savePCDFileASCII ("ndt.pcd", *ndt_cloud);
  communication_middleware::NdtToViewmsg *to_viewmsg =
      new (communication_middleware::NdtToViewmsg);
  ndt_view_msgs::NDTViewArray view_msg_array;
  ndt.getTarget_cells().getDistplayOcPointCloud(Oc_pointcloud);
  ndt.getTarget_cells().getDistplaySingleOcPointCloud(single_oc_pointcloud);
  pcl_update::VoxelGridCovariance<pcl::PointXYZ> &target_cells =
      ndt.getTarget_cells();
  //  cout << "1 "<<endl;
  const std::vector<int> voxel_centroids_leaf_indices =
      target_cells.getVoxel_centroids_leaf_indices_();
//  发送累积的出来的高斯分布
  for (int i = 0; i < voxel_centroids_leaf_indices.size(); i++) {
    int idx = voxel_centroids_leaf_indices[i];
    const pcl_update::VoxelGridCovariance<pcl::PointXYZ>::Leaf *leaf_ptr =
        target_cells.getLeaf(idx);
    if (leaf_ptr) {
      const Eigen::Matrix3d cov = leaf_ptr->getCov();
      const Eigen::Vector3d mean = leaf_ptr->getMean();
      const Eigen::Array4d cube_position = target_cells.GetLeafCenter(idx);
      ndt_view_msgs::NDTView view_msg;
      bool vectory_flag = to_viewmsg->VoxelGridCovarianceToViewmsg(
          cov, mean, i, cube_position, &view_msg);
      if (vectory_flag) view_msg_array.leaves.push_back(view_msg);
    }
  }
  ndt_view_msgs::NDTViewArray view_singlemsg_array;
  //  发送单帧的计算出来的高斯分布
  for (int i = 0; i < voxel_centroids_leaf_indices.size(); i++) {
    int idx = voxel_centroids_leaf_indices[i];
    const pcl_update::VoxelGridCovariance<pcl::PointXYZ>::Leaf *leaf_ptr =
        target_cells.getLeaf(idx);
    if (leaf_ptr) {
      const Eigen::Matrix3d cov = (leaf_ptr->getVector_of_cov()).back();
      const Eigen::Vector3d mean = (leaf_ptr->getVector_of_mean()).back();
      const Eigen::Array4d cube_position = target_cells.GetLeafCenter(idx);
      ndt_view_msgs::NDTView view_msg;
      bool vectory_flag = to_viewmsg->VoxelGridCovarianceToViewmsg(
          cov, mean, i, cube_position, &view_msg);
      if (vectory_flag) view_singlemsg_array.leaves.push_back(view_msg);
    }
  }
  //输出某体素的单帧数据
  if (counter == 100) {
    int output_num = 0;
    while (output_num != -1) {
      cout << "请输入想采集的体素在重心点云中的序号,输入-1结束" << endl;
      cin >> output_num;
      std::string output_mean = "mean_of_" + std::to_string(output_num) + ".txt";
      std::string output_nrpoints = "nrpoints_of_" + std::to_string(output_num) + ".txt";
      std::string output_cov = "cov_of_" + std::to_string(output_num) + ".txt";
      ofstream write;
      write.open("logfiles/" + output_mean, ios::trunc);  //用ios::trunc会删除原本的东西
      write.close();
      write.open("logfiles/" + output_cov, ios::trunc);  //用ios::trunc会删除原本的东西
      write.close();
      write.open("logfiles/" + output_nrpoints, ios::trunc);  //用ios::trunc会删除原本的东西
      write.close();
      const std::vector<int> voxel_centroids_leaf_indices =
          target_cells.getVoxel_centroids_leaf_indices_();
      int idx = voxel_centroids_leaf_indices[output_num];
      const pcl_update::VoxelGridCovariance<pcl::PointXYZ>::Leaf *leaf_ptr =
          target_cells.getLeaf(idx);
      if (leaf_ptr) {
        const Eigen::Matrix3d cov = leaf_ptr->getCov();
        const Eigen::Vector3d mean = leaf_ptr->getMean();
        const std::vector<Eigen::Vector3d> vector_of_mean =
            leaf_ptr->getVector_of_mean();
        const std::vector<Eigen::Matrix3d> vector_of_cov =
            leaf_ptr->getVector_of_cov();
        const std::vector<int> vector_of_nr_points =
            leaf_ptr->getVector_of_nr_points();
        write.open("logfiles/" + output_mean,
                   ios::app);  //用ios::app不会覆盖文件内容
        for (const auto &mean : vector_of_mean) {
          write << setprecision(16) << "[" << mean[0] << "," << mean[1] << ","
                << mean[2] << "]" << "," << endl;
        }
        write.close();
        write.open("logfiles/" + output_cov,
                   ios::app);  //用ios::app不会覆盖文件内容
        for (const auto &cov : vector_of_cov) {
          write << setprecision(16) << cov << endl << endl;
        }
        write.close();
        write.open("logfiles/" + output_nrpoints,
                   ios::app);  //用ios::app不会覆盖文件内容
        for (const auto &nr_points : vector_of_nr_points) {
          write << setprecision(16) << nr_points << endl;
        }
        write.close();
      }
    }
  }

  // TODO store frame_id, x, y, resolution information
  view_msg_array.frame_id = "odom";
  view_msg_array.resolution = ndt.getResolution();
  view_singlemsg_array.frame_id = "odom_single";
  view_singlemsg_array.resolution = ndt.getResolution();
  // Saving Oc cloud.
  // pcl::io::savePCDFileASCII ("Oc.pcd", *Oc_pointcloud);

  //转换成ros消息的格式
//  pcl::toROSMsg(*target_cloud, ori_output);
  pcl::toROSMsg(*Oc_pointcloud, update_output);
  pcl::toROSMsg(*single_oc_pointcloud, update_single_output);
  pcl::toROSMsg(*ndt_cloud, ndt_viz);
  ori_output.header.frame_id = "map";
  update_output.header.frame_id = "odom";
  update_single_output.header.frame_id = "odom_single";
  ndt_viz.header.frame_id = "ndt";

  //发送到output topic
//  pub.publish(ori_output);
  pub.publish(update_output);
  pub.publish(update_single_output);
  pub.publish(ndt_viz);
  pub_ndt_msgs.publish(view_msg_array);
  pub_ndt_msgs.publish(view_singlemsg_array);
  std::string file_name = "point_cloud_" + std::to_string(counter) + ".pcd";
}

int main(int argc, char **argv) {
  //  创建文件夹存储debug文件
  std::string test = "logfiles";
  mkdir(test.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
  ofstream write;
  write.open("logfiles/Oc.txt", ios::trunc);  //用ios::trunc会删除原本的东西
  write.close();
  write.open("logfiles/nr_points.txt",
             ios::trunc);  //用ios::app不会覆盖文件内容
  write.close();
  write.open("logfiles/pt_sum.txt", ios::trunc);  //用ios::trunc会删除原本的东西
  write.close();
  write.open("logfiles/pt3d_pt3dT_.txt",
             ios::trunc);  //用ios::app不会覆盖文件内容
  write.close();
  write.open("logfiles/centroid_sum_.txt",
             ios::trunc);  //用ios::app不会覆盖文件内容
  write.close();
  write.open("logfiles/cov_.txt", ios::trunc);  //用ios::trunc会删除原本的东西
  write.close();
  write.open("logfiles/mean_.txt", ios::trunc);  //用ios::trunc会删除原本的东西
  write.close();
  write.open("logfiles/centroid.txt", ios::trunc);  //用ios::trunc会删除原本的东西
  write.close();
  write.open("logfiles/point_cloud_centroid.txt",
             ios::trunc);  //用ios::app不会覆盖文件内容
  write.close();
  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon(0.01);
//  ndt_ori.setTransformationEpsilon(0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize(0.1);
//  ndt_ori.setStepSize(0.1);
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution(1.0);
//  ndt_ori.setResolution(1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations(60);
//  ndt_ori.setMaximumIterations(60);
  // ros部分
  ros::init(argc, argv, "point_cloud_subscriber");
  ros::NodeHandle node_handle;

  pub_ndt_msgs =
      node_handle.advertise<ndt_view_msgs::NDTViewArray>("NDTView_msg", 1000);
  pub = node_handle.advertise<sensor_msgs::PointCloud2>("output_rviz", 1000);

  ros::Subscriber point_cloud_sub =
      node_handle.subscribe("/points_raw", 1000, SubscribePointCloud);
  ros::spin();

  return (0);
}
