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

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>


#include "ros/ros.h"
using namespace std :: literals :: chrono_literals;
// Initializing Normal Distributions Transform (NDT).
static pcl_update::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_ori;

//原始ndt的结果点云
static pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//updatendt的结果点云
static pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_update(new pcl::PointCloud<pcl::PointXYZ>);
//原始ndt的结果点云发送到rviz上
static sensor_msgs::PointCloud2 ori_output;
//updatendt的结果点云发送到rviz上
static sensor_msgs::PointCloud2 update_output;
//显示ndt地图的点云
static sensor_msgs::PointCloud2 ndt_viz;
//用来显示的Oc点云
static pcl::PointCloud<pcl::PointXYZI>::Ptr Oc_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
//申明发布器
ros::Publisher pub;
static size_t counter = 0;
void
SubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
  //记录分数之差
  static double ori_sum=0,diff_sum=0;
  //记录前一帧位姿
  static Eigen::Matrix4f T_last=ndt.getFinalTransformation();
  static Eigen::Matrix4f T_last_ori=ndt_ori.getFinalTransformation();
  //记录当前位姿
  static Eigen::Matrix4f T_now=ndt.getFinalTransformation();
  static Eigen::Matrix4f T_now_ori=ndt_ori.getFinalTransformation();

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidar_message, *input_cloud);

  counter++;

  // Loading first scan of room.
  if(input_cloud->empty()) {
    // /media/zxd/60787A51787A25C6/pcl_ndt/PointCloud/bin2pcd/velodyne/pcd/1025/
    PCL_ERROR ("Couldn't read file in package \n");
    return;
  }
  std::cout << "Loaded " << input_cloud->size() << " data points from "+std::to_string(counter) << std::endl;

  if (!(counter-1)) {
    *target_cloud = *input_cloud;
    *target_cloud_update = *input_cloud;
    // 设置初始要建立ndt地图的点云.
    ndt.setInputTarget(target_cloud_update);
    ndt_ori.setInputTarget(target_cloud);
    return;
  }
  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //降采样之后的update点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_update(new pcl::PointCloud<pcl::PointXYZ>);
  //降采样之后的ori点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud(input_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
  //将要加入结果点云的数据进行降采样
  approximate_voxel_filter.setLeafSize(0.01, 0.01, 0.01);
  approximate_voxel_filter.filter(*filtered_cloud_update);
  approximate_voxel_filter.filter(*filtered_cloud_ori);
  std::cout << "Filtered cloud contains " << filtered_cloud_update->size()
            << " data points from room_scan2.pcd" << std::endl;
  // Setting point cloud to be aligned.
  ndt.setInputSource(filtered_cloud);
  ndt_ori.setInputSource(filtered_cloud);
  //按照匀速模型设置初始位姿
  Eigen::Matrix4f init_guess = T_now*T_last.inverse()*T_now;
  Eigen::Matrix4f init_guess_ori = T_now_ori*T_last_ori.inverse()*T_now_ori;

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);

  //开始匹配
  start=clock();		//程序开始计时
  ndt.align(*output_cloud, init_guess);
  end=clock();		//程序结束计时
  double endtime=(double)(end-start)/CLOCKS_PER_SEC;
  ofstream write;
  write.open("update_align_time.txt", ios::app);                //用ios::app不会覆盖文件内容
  write<< endtime*1000 << endl;
  write.close();
  //std::cout<<"update_align time:"<<endtime*1000<<"ms"<<std::endl;	//ms为单位

  start=clock();		//程序开始计时
  ndt_ori.align(*output_cloud_ori, init_guess_ori);
  end=clock();		//程序结束计时
  endtime=(double)(end-start)/CLOCKS_PER_SEC;
  write.open("ori_align_time.txt", ios::app);                //用ios::app不会覆盖文件内容
  write<< endtime*1000 << endl;
  write.close();
  //std::cout<<"ori_align time:"<<endtime*1000<<"ms"<<std::endl;	//ms为单位

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
            << " update_score: " << ndt.getFitnessScore() << std::endl;
  std::cout << "Normal Distributions Transform has converged:" << ndt_ori.hasConverged()
            << " ori_score: " << ndt_ori.getFitnessScore() << std::endl;
  ori_sum = ndt_ori.getFitnessScore();
  diff_sum = diff_sum + (ndt.getFitnessScore() - ndt_ori.getFitnessScore()) / ori_sum * 100;
  cout << "diff = " << diff_sum << "%" << endl;
  //更新上一帧和本帧位姿

  T_last=T_now;
  T_last_ori=T_now_ori;
  //TODO 将原始位姿作为新点云的转换，目的是为了观察位姿相同的情况下的均值方差，之后需要改回来
  T_now = ndt_ori.getFinalTransformation();
  T_now_ori = ndt_ori.getFinalTransformation();

  // Transforming unfiltered, input cloud using found transform.
  //TODO 将原始位姿作为新点云的转换，目的是为了观察位姿相同的情况下的均值方差，之后需要改回来
  pcl::transformPointCloud(*filtered_cloud_update, *output_cloud, ndt_ori.getFinalTransformation());
  pcl::transformPointCloud(*filtered_cloud_ori, *output_cloud_ori, ndt_ori.getFinalTransformation());

  // 更新ndt地图
  start=clock();		//程序开始计时
  ndt.updateInputTarget(output_cloud);
  end=clock();		//程序结束计时
  endtime=(double)(end-start)/CLOCKS_PER_SEC;
  write.open("update_mapping_time.txt", ios::app);                //用ios::app不会覆盖文件内容
  write<< endtime*1000 << endl;
  write.close();
  //std::cout<<"update_mapping time:"<<endtime*1000<<"ms"<<std::endl;	//ms为单位

  // Saving result cloud.
//  pcl::io::savePCDFileASCII ("output.pcd", *output_cloud);

  //将转换后的地图加入到
  *target_cloud = *target_cloud + *output_cloud_ori;
  *target_cloud_update = *target_cloud_update + *output_cloud;

  start=clock();		//程序开始计时
  ndt_ori.setInputTarget(target_cloud);
  end=clock();		//程序结束计时
  endtime=(double)(end-start)/CLOCKS_PER_SEC;
  write.open("ori_mapping_time.txt", ios::app);                //用ios::app不会覆盖文件内容
  write<< endtime*1000 << endl;
  write.close();
  //std::cout<<"ori_mapping time:"<<endtime*1000<<"ms"<<std::endl;	//ms为单位

  //用点云表示ndt地图
  pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.getTarget_cells().getDisplayCloud(*ndt_cloud);

  // Saving ndt cloud.
  //pcl::io::savePCDFileASCII ("ndt.pcd", *ndt_cloud);

  ndt.getTarget_cells().getDistplayOcPointCloud(Oc_pointcloud);
  // Saving Oc cloud.
  //pcl::io::savePCDFileASCII ("Oc.pcd", *Oc_pointcloud);

  //转换成ros消息的格式
  //pcl::toROSMsg(*target_cloud, ori_output);
  pcl::toROSMsg(*Oc_pointcloud, update_output);
  pcl::toROSMsg(*ndt_cloud, ndt_viz);
  ori_output.header.frame_id = "odom";
  update_output.header.frame_id = "odom";
  ndt_viz.header.frame_id = "ndt";

  //发送到output topic
  //pub.publish(ori_output);
  pub.publish(update_output);
  pub.publish(ndt_viz);
  std::string file_name = "point_cloud_" + std::to_string(counter) + ".pcd";
}

int
main (int argc, char** argv)
{
  ofstream write;
  write.open("Oc.txt", ios::trunc);                //用ios::app不会覆盖文件内容
  write.close();
  write.open("nr_points.txt", ios::trunc);                //用ios::app不会覆盖文件内容
  write.close();
  write.open("pt_sum.txt", ios::trunc);                //用ios::app不会覆盖文件内容
  write.close();
  write.open("pt3d_pt3dT_.txt", ios::trunc);                //用ios::app不会覆盖文件内容
  write.close();
  write.open("centroid_sum_.txt", ios::trunc);                //用ios::app不会覆盖文件内容
  write.close();
  write.open("cov_.txt", ios::trunc);                //用ios::app不会覆盖文件内容
  write.close();
  write.open("mean_.txt", ios::trunc);                //用ios::app不会覆盖文件内容
  write.close();
  write.open("centroid.txt", ios::trunc);                //用ios::app不会覆盖文件内容
  write.close();
  write.open("point_cloud_centroid.txt", ios::trunc);                //用ios::app不会覆盖文件内容
  write.close();
  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon(0.01);
  ndt_ori.setTransformationEpsilon(0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize(0.1);
  ndt_ori.setStepSize(0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution(1.0);
  ndt_ori.setResolution(1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations(60);
  ndt_ori.setMaximumIterations(60);
  //ros部分
    ros::init(argc, argv, "point_cloud_subscriber");
  ros::NodeHandle node_handle;


  pub = node_handle.advertise<sensor_msgs::PointCloud2> ("output_rviz", 1000);

  ros::Subscriber point_cloud_sub =
          node_handle.subscribe("/points_raw", 1000, SubscribePointCloud);
  ros::spin();

  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
          viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          target_color (target_cloud, 0, 0, 255);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters ();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped ())
    {
        viewer_final->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    return (0);
}
