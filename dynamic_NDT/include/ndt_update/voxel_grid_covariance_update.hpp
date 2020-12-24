/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_VOXEL_GRID_COVARIANCE_IMPL_H_
#define PCL_VOXEL_GRID_COVARIANCE_IMPL_H_

#include <pcl/common/common.h>
#include <pcl/common/point_tests.h> // for isXYZFinite
//#include <pcl/filters/voxel_grid_covariance.h>
#include "voxel_grid_covariance_update.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues> // for SelfAdjointEigenSolver
#include <boost/mpl/size.hpp> // for size
#include <boost/random/mersenne_twister.hpp> // for mt19937
#include <boost/random/normal_distribution.hpp> // for normal_distribution
#include <boost/random/variate_generator.hpp> // for variate_generator
#include <iomanip>

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl_update::VoxelGridCovariance<PointT>::applyFilter (PointCloud &output)
{
  voxel_centroids_leaf_indices_.clear ();

  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height = 1;                          // downsampling breaks the organized structure
  output.is_dense = true;                     // we filter out invalid points
  output.clear ();

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    pcl::getMinMax3D<PointT> (input_, filter_field_name_, static_cast<float> (filter_limit_min_), static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
  else
    pcl::getMinMax3D<PointT> (*input_, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
  std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
  std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

  if((dx*dy*dz) > std::numeric_limits<std::int32_t>::max())
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
    output.clear();
    return;
  }

  //定义一个变量表示每次扩张2倍之后的最小值坐标
  Eigen::Vector4i min_b_power_ (-1, -1, -1, 0) ,max_b_power_ (1, 1, 1, 0);
  for (int i = 0; i < 32; i++) {
    if (min_b_power_[0]*leaf_size_[0]>min_p[0]) {
      min_b_power_[0]<<=1;
    }
    if (min_b_power_[1]*leaf_size_[1]>min_p[1]) {
      min_b_power_[1]<<=1;
    }
    if (min_b_power_[2]*leaf_size_[2]>min_p[2]) {
      min_b_power_[2]<<=1;
    }
    if (max_b_power_[0]*leaf_size_[0]<max_p[0]) {
      max_b_power_[0]<<=1;
    }
    if (max_b_power_[1]*leaf_size_[1]<max_p[1]) {
      max_b_power_[1]<<=1;
    }
    if (max_b_power_[2]*leaf_size_[2]<max_p[2]) {
      max_b_power_[2]<<=1;
    }
  }
  // Compute the minimum and maximum bounding box values
  min_b_=min_b_power_;
  max_b_=max_b_power_;

  // Compute the minimum and maximum bounding box values
//  min_b_[0] = static_cast<int> (std::floor (min_p[0] * inverse_leaf_size_[0]));
//  max_b_[0] = static_cast<int> (std::floor (max_p[0] * inverse_leaf_size_[0]));
//  min_b_[1] = static_cast<int> (std::floor (min_p[1] * inverse_leaf_size_[1]));
//  max_b_[1] = static_cast<int> (std::floor (max_p[1] * inverse_leaf_size_[1]));
//  min_b_[2] = static_cast<int> (std::floor (min_p[2] * inverse_leaf_size_[2]));
//  max_b_[2] = static_cast<int> (std::floor (max_p[2] * inverse_leaf_size_[2]));
  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  // Clear the leaves
  leaves_.clear ();

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

  UpdateLeaves(output);

  output.width = output.size ();
  std::cout<<"filter output.width = " <<output.width<<std::endl;
  std::cout<<"filter leaves_size = "<<std::endl<<leaves_.size()<<std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl_update::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint (const Eigen::Matrix<int, 3, Eigen::Dynamic>& relative_coordinates, const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  neighbors.clear ();

  // Find displacement coordinates
  Eigen::Vector4i ijk = Eigen::floor(reference_point.getArray4fMap() * inverse_leaf_size_).template cast<int>();
  ijk[3] = 0;
  const Eigen::Array4i diff2min = min_b_ - ijk;
  const Eigen::Array4i diff2max = max_b_ - ijk;
  neighbors.reserve (relative_coordinates.cols ());

  // Check each neighbor to see if it is occupied and contains sufficient points
  for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)
  {
    const Eigen::Vector4i displacement = (Eigen::Vector4i () << relative_coordinates.col (ni), 0).finished ();
    // Checking if the specified cell is in the grid
    if ((diff2min <= displacement.array ()).all () && (diff2max >= displacement.array ()).all ())
    {
      const auto leaf_iter = leaves_.find (((ijk + displacement - min_b_).dot (divb_mul_)));
      if (leaf_iter != leaves_.end () && leaf_iter->second.nr_points >= min_points_per_voxel_)
      {
        LeafConstPtr leaf = &(leaf_iter->second);
        neighbors.push_back (leaf);
      }
    }
  }

  return static_cast<int> (neighbors.size());
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl_update::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint (const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  Eigen::MatrixXi relative_coordinates = pcl::getAllNeighborCellIndices();
  return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl_update::VoxelGridCovariance<PointT>::getVoxelAtPoint(const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  return getNeighborhoodAtPoint(Eigen::Matrix<int, 3, Eigen::Dynamic>::Zero(3,1), reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl_update::VoxelGridCovariance<PointT>::getFaceNeighborsAtPoint(const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  Eigen::Matrix<int, 3, Eigen::Dynamic> relative_coordinates(3, 7);
  relative_coordinates.setZero();
  relative_coordinates(0, 1) = 1;
  relative_coordinates(0, 2) = -1;
  relative_coordinates(1, 3) = 1;
  relative_coordinates(1, 4) = -1;
  relative_coordinates(2, 5) = 1;
  relative_coordinates(2, 6) = -1;

  return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl_update::VoxelGridCovariance<PointT>::getAllNeighborsAtPoint(const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  Eigen::Matrix<int, 3, Eigen::Dynamic> relative_coordinates(3, 27);
  relative_coordinates.col(0).setZero();
  relative_coordinates.rightCols(26) = pcl::getAllNeighborCellIndices();

  return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl_update::VoxelGridCovariance<PointT>::getDisplayCloud (pcl::PointCloud<pcl::PointXYZ>& cell_cloud)
{
  cell_cloud.clear ();

  int pnt_per_cell = 2000;
  boost::mt19937 rng;
  boost::normal_distribution<> nd (0.0, leaf_size_.head (3).norm ());
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

  Eigen::LLT<Eigen::Matrix3d> llt_of_cov;
  Eigen::Matrix3d cholesky_decomp;
  Eigen::Vector3d cell_mean;
  Eigen::Vector3d rand_point;
  Eigen::Vector3d dist_point;

  // Generate points for each occupied voxel with sufficient points.
  for (typename std::map<std::size_t, Leaf>::iterator it = leaves_.begin (); it != leaves_.end (); ++it)
  {
    Leaf& leaf = it->second;

    if (leaf.nr_points >= min_points_per_voxel_)
    {
      cell_mean = leaf.mean_;
      llt_of_cov.compute (leaf.cov_);
      cholesky_decomp = llt_of_cov.matrixL ();

      double Oc = leaf.getOc();
      // Random points generated by sampling the normal distribution given by voxel mean and covariance matrix
      for (int i = 0; i < pnt_per_cell * Oc; i++)
      {
        rand_point = Eigen::Vector3d (var_nor (), var_nor (), var_nor ());
        dist_point = cell_mean + cholesky_decomp * rand_point;
        cell_cloud.push_back (pcl::PointXYZ (static_cast<float> (dist_point (0)), static_cast<float> (dist_point (1)), static_cast<float> (dist_point (2))));
      }
    }
  }
}

/********************************************************改动部分****************************************************/

template<typename PointT> void
pcl_update::VoxelGridCovariance<PointT>::getDistplayOcPointCloud (const pcl::PointCloud<pcl::PointXYZI>::Ptr &displaypointcloud){
  displaypointcloud -> clear();
  displaypointcloud->height = 1;
  int cnt = 100;
  for (typename std::map<std::size_t, Leaf>::iterator it = leaves_.begin (); it != leaves_.end (); ++it)
  {
    Leaf& leaf = it->second;
    double Oc = leaf.getOc();
    for (const auto& point: *(leaf._Pc))
    {
      //point.intensity = leaf.getOc();
      displaypointcloud -> push_back(pcl::PointXYZI(point.x,point.y,point.z,Oc));
    }
    cnt--;
  }
  displaypointcloud->width = displaypointcloud->size ();
}

template<typename PointT> void
pcl_update::VoxelGridCovariance<PointT>::OcUpdate() {
  std::shared_ptr<ComputeProbability> computer(new ComputeProbability(input_));


  int pnt_per_cell = 100;
  boost::mt19937 rng;
  boost::normal_distribution<> nd (0.0, leaf_size_.head (3).norm ());
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

  Eigen::LLT<Eigen::Matrix3d> llt_of_cov;
  Eigen::Matrix3d cholesky_decomp;
  Eigen::Vector3d cell_mean;
  Eigen::Vector3d rand_point;
  Eigen::Vector3d dist_point;

  // Generate points for each occupied voxel with sufficient points.
  for (typename std::map<std::size_t, Leaf>::iterator it = leaves_.begin (); it != leaves_.end (); ++it)
  {
    Leaf& leaf = it->second;

    if (leaf.nr_points >= min_points_per_voxel_)
    {
      cell_mean = leaf.mean_;
      llt_of_cov.compute (leaf.cov_);
      cholesky_decomp = llt_of_cov.matrixL ();

      double Oc = leaf.getOc();
      double Pr_mean = 0;
      // 按照对应格的均值和方差进行随机采样，并且计算每个leaf对应的平均概率值赋给Oc
      for (int i = 0; i < pnt_per_cell; i++)
      {
        rand_point = Eigen::Vector3d (var_nor (), var_nor (), var_nor ());
        dist_point = cell_mean + cholesky_decomp * rand_point;
        //计算每个随机点所对应的概率值
        double Pr = computer->compute_PointProbability (pcl::PointXYZ (static_cast<float> (dist_point (0)),
                                                                       static_cast<float> (dist_point (1)),
                                                                       static_cast<float> (dist_point (2))));
        Pr = fmin(0.9,fmax(0.3,Pr));
        Pr_mean += Pr;
      }
      Pr_mean /= pnt_per_cell;
      double logit_Oc = log(Oc / (1 - Oc)) + log(Pr_mean / (1 - Pr_mean));
      Oc = 1 / (1 + exp(-logit_Oc));
      Oc = fmin(0.9,fmax(0.1,Oc));
      leaf.setOc(Oc);
    }
  }
}

template<typename PointT> void
pcl_update::VoxelGridCovariance<PointT>::addPointToLeavesCurr (const PointT &point,
                                                               std::map<std::size_t, Leaf> &leaves_curr,
                                                               int centroid_size){
  // Compute the centroid leaf index
  const Eigen::Vector4i ijk =
          Eigen::floor(point.getArray4fMap() * inverse_leaf_size_.array())
                  .template cast<int>();
  // divb_mul_[3] = 0 by assignment
  int idx = (ijk - min_b_).dot(divb_mul_);

//    Leaf& leaf = leaves_[idx];
  Leaf &leaf = leaves_curr[idx];
  if (leaf.nr_points == 0) {
    leaf.centroid.resize(centroid_size);
    leaf.centroid.setZero();
  }

  Eigen::Vector3d pt3d = point.getVector3fMap().template cast<double>();
  // Accumulate point sum for centroid calculation
//  leaf.pt_sum += pt3d;

  if (leaf.nr_points > 0) {
    leaf.cov_ = 1.0 * (leaf.nr_points - 1) / leaf.nr_points * leaf.cov_ +
                1.0 / (leaf.nr_points + 1) * (leaf.mean_ - pt3d) * (leaf.mean_ - pt3d).transpose();
  }
  leaf.mean_ = 1.0 * leaf.nr_points / (leaf.nr_points + 1) * leaf.mean_ + 1.0 / (leaf.nr_points + 1) * pt3d;

  // Accumulate x*xT for single pass covariance calculation
//  leaf.pt3d_pt3dT_ += pt3d * pt3d.transpose();
  // Do we need to process all the fields?
  if (!downsample_all_data_) {
    leaf.centroid.template head<3>() = 1.0 * leaf.nr_points / (leaf.nr_points + 1) * leaf.centroid.template head<3>() +
                                       1.0 / (leaf.nr_points + 1) * point.getVector3fMap();
  } else {
    // Copy all the fields
    Eigen::VectorXf centroid = Eigen::VectorXf::Zero(centroid_size);
    pcl::for_each_type<FieldList>(pcl::NdCopyPointEigenFunctor<PointT>(point, centroid));
    leaf.centroid.template head<3>() = 1.0 * leaf.nr_points / (leaf.nr_points + 1) * leaf.centroid +
                                       1.0 / (leaf.nr_points + 1) * centroid;
  }
  ++leaf.nr_points;
  //在体素对应的点云中加入对应的点
  leaf._Pc->push_back(point);
}

template<typename PointT> void
pcl_update::VoxelGridCovariance<PointT>::addLeavesCurrToLeaves (const std::map<std::size_t, Leaf> &leaves_curr,
                                                                PointCloud &output,
                                                                int centroid_size,
                                                                int rgba_index){
  output.reserve(leaves_.size());
  if (searchable_)
    voxel_centroids_leaf_indices_.reserve(leaves_.size());
  int cp = 0;
  if (save_leaf_layout_)
    leaf_layout_.resize(div_b_[0] * div_b_[1] * div_b_[2], -1);

  // Eigen values and vectors calculated to prevent near singluar matrices
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
  Eigen::Matrix3d eigen_val;
  //Eigen::Vector3d pt_sum;

  // Eigen values less than a threshold of max eigen value are inflated to a set fraction of the max eigen value.
  double min_covar_eigvalue;
  for (typename std::map<std::size_t, Leaf>::const_iterator it = leaves_curr.begin(); it != leaves_curr.end(); ++it) {
    const Leaf &leaf_curr = it->second;
    const auto it_leaves = leaves_.find(it->first);
    if (it_leaves == leaves_.end()) {
      Leaf &leaf = leaves_[it->first];
      leaf.centroid.resize(centroid_size);
      leaf.centroid.setZero();
    }
    Leaf &leaf = leaves_[it->first];
    *(leaf._Pc) += *(leaf_curr._Pc);
    const int nr_pointsNow = leaf.nr_points + leaf_curr.nr_points;
    // Normalize the centroid
    const float factorOfcentroid_ = 1.0 * leaf.nr_points / nr_pointsNow;
    const float factorOfcentroid_curr = 1.0 * leaf_curr.nr_points / nr_pointsNow;
    leaf.centroid = factorOfcentroid_ * leaf.centroid + factorOfcentroid_curr * leaf_curr.centroid;


    // Single pass covariance calculation
    if (leaf.nr_points > 0 && leaf_curr.nr_points > 0) {
      const double factorOfcov_ = 1.0 * (leaf.nr_points - 1) / (nr_pointsNow - 1);
      const double factorOfcov_curr = 1.0 * (leaf_curr.nr_points - 1) / (nr_pointsNow - 1);
      const double factorOfmean = 1.0 * leaf_curr.nr_points * leaf.nr_points /
                                  (nr_pointsNow *
                                  (nr_pointsNow - 1));
      leaf.cov_ = factorOfcov_ * leaf.cov_ +
                   factorOfcov_curr * leaf_curr.cov_ +
                   factorOfmean * (leaf.mean_ - leaf_curr.mean_) * (leaf.mean_ - leaf_curr.mean_).transpose();
    } else leaf.cov_ += leaf_curr.cov_;

    // Normalize mean
    const double factorOfmean_ = 1.0 * leaf.nr_points / nr_pointsNow;
    const double factorOfmean_curr = 1.0 * leaf_curr.nr_points / nr_pointsNow;
    leaf.mean_ = factorOfmean_ * leaf.mean_ + factorOfmean_curr * leaf_curr.mean_;

    // If the voxel contains sufficient points, its covariance is calculated and is added to the voxel centroids and output clouds.
    // Points with less than the minimum points will have a can not be accuratly approximated using a normal distribution.
    if (nr_pointsNow >= min_points_per_voxel_) {
      if (save_leaf_layout_)
        leaf_layout_[it->first] = cp++;
      if (voxel_leaf_centroids_indices_.find(it->first) == voxel_leaf_centroids_indices_.end()) {
        //如果体素还不存在于重心点云中，则需要在重心点云中加入体素对应的重心点
        voxel_leaf_centroids_indices_[it->first] = output.size();
        output.push_back(PointT());
        // Stores the voxel indice for fast access searching
        if (searchable_) voxel_centroids_leaf_indices_.push_back(static_cast<int> (it->first));

        //TODO delete 为了保持和原pcl点云库中算法一致，之后库中修复这个bug之后要把这句删掉
        leaf.cov_ += Eigen::Matrix3d::Identity() / (nr_pointsNow - 1);
      }
      const int centroid_index = voxel_leaf_centroids_indices_[it->first];
      PointT &centroid = output[centroid_index];
      // Do we need to process all the fields?
      if (!downsample_all_data_) {
        centroid.x = leaf.centroid[0];
        centroid.y = leaf.centroid[1];
        centroid.z = leaf.centroid[2];
      } else {
        pcl::for_each_type<FieldList>(pcl::NdCopyEigenPointFunctor<PointT>(leaf.centroid, centroid));
        // ---[ RGB special case
        if (rgba_index >= 0) {
          pcl::RGB &rgb = *reinterpret_cast<pcl::RGB *> (reinterpret_cast<char *> (&output.back()) + rgba_index);
          rgb.a = leaf.centroid[centroid_size - 4];
          rgb.r = leaf.centroid[centroid_size - 3];
          rgb.g = leaf.centroid[centroid_size - 2];
          rgb.b = leaf.centroid[centroid_size - 1];
        }
      }
      //Normalize Eigen Val such that max no more than 100x min.
      eigensolver.compute(leaf.cov_);
      eigen_val = eigensolver.eigenvalues().asDiagonal();
      leaf.evecs_ = eigensolver.eigenvectors();

      if (eigen_val(0, 0) < 0 || eigen_val(1, 1) < 0 || eigen_val(2, 2) <= 0) {
        leaf.nr_points = -1;
        continue;
      }

      // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]

      min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val(2, 2);
      if (eigen_val(0, 0) < min_covar_eigvalue) {
        eigen_val(0, 0) = min_covar_eigvalue;

        if (eigen_val(1, 1) < min_covar_eigvalue) {
          eigen_val(1, 1) = min_covar_eigvalue;
        }

        leaf.cov_ = leaf.evecs_ * eigen_val * leaf.evecs_.inverse();
      }
      leaf.evals_ = eigen_val.diagonal();

      leaf.icov_ = leaf.cov_.inverse();
      if (leaf.icov_.maxCoeff() == std::numeric_limits<float>::infinity() ||
          leaf.icov_.minCoeff() == -std::numeric_limits<float>::infinity()) {
        leaf.nr_points = -1;
      }
    }
    // 更新点的个数
    leaf.nr_points = nr_pointsNow;
  }
}

template<typename PointT> void
pcl_update::VoxelGridCovariance<PointT>::UpdateLeaves(PointCloud &output) {
  int centroid_size = 4;

  if (downsample_all_data_)
    centroid_size = boost::mpl::size<FieldList>::value;
  // ---[ RGB special case
  std::vector<pcl::PCLPointField> fields;
  int rgba_index = -1;
  rgba_index = pcl::getFieldIndex<PointT> ("rgb", fields);
  if (rgba_index == -1)
    rgba_index = pcl::getFieldIndex<PointT> ("rgba", fields);
  if (rgba_index >= 0)
  {
    rgba_index = fields[rgba_index].offset;
    centroid_size += 4;
  }
  //定义一个leaves_curr表示当前帧点云所对应的体素
  std::map<std::size_t, Leaf> leaves_curr;
  leaves_curr.clear();
  // First pass: go over all points and insert them into the right leaf
  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ()) {
    // Get the distance field index
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex<PointT> (filter_field_name_, fields);
    if (distance_idx == -1)
      PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);

    // First pass: go over all points and insert them into the right leaf
    for (const auto& point: *input_) {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!isXYZFinite(point))
          continue;

      // Get the distance value
      //const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*> (&point);
      const auto pt_data = reinterpret_cast<const std::uint8_t *> (&point);
      float distance_value = 0;
      memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof(float));

      if (filter_limit_negative_) {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
          continue;
      } else {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
          continue;
      }
      addPointToLeavesCurr(point,leaves_curr,centroid_size);
    }
    addLeavesCurrToLeaves(leaves_curr,output,centroid_size,rgba_index);
  } else {// No distance filtering, process all data
    for (const auto &point: *input_) {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!isXYZFinite(point))
          continue;
      addPointToLeavesCurr(point,leaves_curr,centroid_size);
    }

    // Second pass: go over all leaves and compute centroids and covariance matrices
    addLeavesCurrToLeaves(leaves_curr,output,centroid_size,rgba_index);
  }
}

template<typename PointT> void
pcl_update::VoxelGridCovariance<PointT>::applyUpdate (PointCloud &output){
  // Has the input dataset been set already?
  if (!input_) {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    return;
  }
  // 是否已经有栅格了？
  if (getVoxel_centroids_leaf_indices_().size()==0) {
    //如果还没有
    // TODO
    PCL_WARN ("[pcl::%s::applyUpdate] No targets now!\n", getClassName ().c_str ());
    return;
  }
  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height = 1;                          // downsampling breaks the organized structure
  output.is_dense = true;                     // we filter out invalid points
//  output.clear ();
  //点的最大最小值
  Eigen::Vector4f min_p, max_p;
  pcl::getMinMax3D<PointT> (*input_, min_p, max_p);
  //定义一个变量表示每次扩张2倍之后的最小值坐标
  Eigen::Vector4i min_b_power_=min_b_ ,max_b_power_ =max_b_;
  //循环判断最大最小坐标是否要更新
  for (int i = 0; i < 32; i++) {
    if (min_b_power_[0]*leaf_size_[0]>min_p[0]) {
        min_b_power_[0]<<=1;
    }
    if (min_b_power_[1]*leaf_size_[1]>min_p[1]) {
        min_b_power_[1]<<=1;
    }
    if (min_b_power_[2]*leaf_size_[2]>min_p[2]) {
        min_b_power_[2]<<=1;
    }
    if (max_b_power_[0]*leaf_size_[0]<max_p[0]) {
        max_b_power_[0]<<=1;
    }
    if (max_b_power_[1]*leaf_size_[1]<max_p[1]) {
        max_b_power_[1]<<=1;
    }
    if (max_b_power_[2]*leaf_size_[2]<max_p[2]) {
        max_b_power_[2]<<=1;
    }
  }
  // Check that the leaf size is not too small, given the size of the data
  std::int64_t dx = static_cast<std::int64_t>(max_b_power_[0] - min_b_power_[0])+1;
  std::int64_t dy = static_cast<std::int64_t>(max_b_power_[1] - min_b_power_[1])+1;
  std::int64_t dz = static_cast<std::int64_t>(max_b_power_[2] - min_b_power_[2])+1;

  if((dx*dy*dz) > std::numeric_limits<std::int32_t>::max())
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
    output.clear();
    return;
  }

  //更新index
  // Compute the number of divisions needed along all axis
  Eigen::Vector4i div_b_now_;
  div_b_now_ = max_b_power_ - min_b_power_ + Eigen::Vector4i::Ones ();
  div_b_now_[3] = 0;
  Eigen::Vector4i divb_mul_new_(1, div_b_now_[0], div_b_now_[0] * div_b_now_[1], 0);
  //判断格数大小是否发生了变化，如发生变化则需要更新index
  if (div_b_now_[0]!=div_b_[0] ||
      div_b_now_[1]!=div_b_[1] ||
      div_b_now_[2]!=div_b_[2]) {
    //TODO
    std::cout<<"进入更新操作"<<std::endl;
    std::cout<<"leaves_size before = "<<std::endl<<leaves_.size()<<std::endl;
    std::cout<<"voxel_centroids_leaf_indices_size before = "<<std::endl<<voxel_centroids_leaf_indices_.size()<<std::endl;
    //std::cout<<"div_b_now_ = "<<std::endl<<div_b_now_<<std::endl;
    //std::cout<<"div_b_ = "<<std::endl<<div_b_<<std::endl;
    std::cout<<"voxel_leaf_centroids_indices_size before = "<<std::endl<<voxel_leaf_centroids_indices_.size()<<std::endl;
    //定义临时变量存放每个ndt体素的新index
    std::map<std::size_t, Leaf> map_temp;
    std::map<int, int> voxel_leaf_centroids_indices_temp;
    //更新leaves的index值
    for (typename std::map<std::size_t, Leaf>::iterator it = leaves_.begin (); it != leaves_.end (); ++it) {
        int index_temp =it -> first;
        int new_index = Compute_New_index(index_temp, divb_mul_new_, min_b_power_);
        const auto& voxel_leaf_centroids_indices_it = voxel_leaf_centroids_indices_.find(it->first);
        if (voxel_leaf_centroids_indices_it !=
            voxel_leaf_centroids_indices_.end()) {
          const auto& centroid_index = voxel_leaf_centroids_indices_[it->first];
          voxel_leaf_centroids_indices_temp.insert(std::make_pair(new_index,centroid_index));
        }
        //map_temp[new_index] = std::move(it->second);
        map_temp.insert(std::make_pair(new_index,it->second));
    }
    leaves_ = map_temp;
    voxel_leaf_centroids_indices_ = voxel_leaf_centroids_indices_temp;
    // 更新voxel_centroids_leaf_indices
    for (auto &index :voxel_centroids_leaf_indices_) {
      index = Compute_New_index(index, divb_mul_new_, min_b_power_);
    }
    std::cout<<"leaves_size after = "<<std::endl<<leaves_.size()<<std::endl;
    std::cout<<"voxel_centroids_leaf_indices_size after = "<<std::endl<<voxel_centroids_leaf_indices_.size()<<std::endl;
    std::cout<<"voxel_leaf_centroids_indices_size after = "<<std::endl<<voxel_leaf_centroids_indices_.size()<<std::endl;
  }
  //更新原点坐标
  min_b_ = min_b_power_;
  max_b_ = max_b_power_;
  //更新各方向上的体素数量和计算index的乘数
  div_b_ = div_b_now_;
  divb_mul_ = divb_mul_new_;
  UpdateLeaves(output);
  //TODO delete
  static int frame_count = 2;
  ofstream write;
  write.open("nr_points.txt", ios::app);                //用ios::app不会覆盖文件内容
  write << endl << endl << endl;
  write << frame_count << endl;
  write.close();
  write.open("pt_sum.txt", ios::app);                //用ios::app不会覆盖文件内容
  write << endl << endl << endl;
  write << frame_count << endl;
  write.close();
  write.open("pt3d_pt3dT_.txt", ios::app);                //用ios::app不会覆盖文件内容
  write << endl << endl << endl;
  write << frame_count << endl;
  write.close();
  write.open("centroid_sum_.txt", ios::app);                //用ios::app不会覆盖文件内容
  write << endl << endl << endl;
  write << frame_count << endl;
  write.close();
  write.open("cov_.txt", ios::app);                //用ios::app不会覆盖文件内容
  write << endl << endl << endl;
  write << frame_count << endl;
  write.close();
  write.open("mean_.txt", ios::app);                //用ios::app不会覆盖文件内容
  write << endl << endl << endl;
  write << frame_count << endl;
  write.close();
  write.open("centroid.txt", ios::app);                //用ios::app不会覆盖文件内容
  write << endl << endl << endl;
  write << frame_count << endl;
  write.close();
  write.open("point_cloud_centroid.txt", ios::app);                //用ios::app不会覆盖文件内容
  write << endl << endl << endl;
  write << frame_count << endl;
  write.close();
  frame_count++;
  int i = 0, t = 1;
  for (typename std::map<std::size_t, Leaf>::iterator it = leaves_.begin (); it != leaves_.end (); ++it,++i)
  {
    if (i >= (int)(leaves_.size() * (0.3 * t)) &&
                   it->second.nr_points > min_points_per_voxel_) {
      const Leaf &leaf = it->second;
      //cout << "nr_points = " << leaf.nr_points << endl;
      write.open("nr_points.txt", ios::app);                //用ios::app不会覆盖文件内容
      write << leaf.nr_points << endl;
      write.close();
      write.open("pt_sum.txt", ios::app);                //用ios::app不会覆盖文件内容
//      write << setprecision(16) << leaf.pt_sum << endl << endl;
//      write.close();
//      write.open("pt3d_pt3dT_.txt", ios::app);                //用ios::app不会覆盖文件内容
//      write << setprecision(16) << leaf.pt3d_pt3dT_ + Eigen::Matrix3d::Identity() << endl << endl;
//      write.close();
//      write.open("centroid_sum_.txt", ios::app);                //用ios::app不会覆盖文件内容
//      write << setprecision(16) << leaf.centroid_sum_ << endl << endl;
//      write.close();
      write.open("cov_.txt", ios::app);                //用ios::app不会覆盖文件内容
      write << setprecision(16) << leaf.cov_ << endl << endl;
      write.close();
      write.open("mean_.txt", ios::app);                //用ios::app不会覆盖文件内容
      write << setprecision(16) << leaf.mean_ << endl << endl;
      write.close();
      write.open("centroid.txt", ios::app);                //用ios::app不会覆盖文件内容
      write << setprecision(16) << leaf.centroid << endl << endl;
      write.close();

      const auto &point = (*voxel_centroids_)[voxel_leaf_centroids_indices_[it->first]];
      Eigen::Vector3d pt3d = point.getVector3fMap().template cast<double>();
      write.open("point_cloud_centroid.txt", ios::app);                //用ios::app不会覆盖文件内容
      write << setprecision(16) << pt3d << endl << endl;
      write.close();
      t++;
    }
  }
  OcUpdate();
  output.width = output.size ();
}


/********************************************************改动部分****************************************************/
#define PCL_INSTANTIATE_VoxelGridCovariance(T) template class PCL_EXPORTS pcl_update::VoxelGridCovariance<T>;

#endif    // PCL_VOXEL_GRID_COVARIANCE_IMPL_H_
