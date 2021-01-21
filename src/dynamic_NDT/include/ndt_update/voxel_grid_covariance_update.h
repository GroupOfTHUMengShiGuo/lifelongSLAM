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

#pragma once

#include <pcl/filters/voxel_grid.h>
#include <map>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ComputeProbability.h>
namespace pcl_update
{
  /** \brief A searchable voxel strucure containing the mean and covariance of the data.
    * \note For more information please see
    * <b>Magnusson, M. (2009). The Three-Dimensional Normal-Distributions Transform —
    * an Efﬁcient Representation for Registration, Surface Analysis, and Loop Detection.
    * PhD thesis, Orebro University. Orebro Studies in Technology 36</b>
    * \author Brian Okorn (Space and Naval Warfare Systems Center Pacific)
    */
  template<typename PointT>
class VoxelGridCovariance : public pcl::VoxelGrid<PointT>
  {
    protected:
      using pcl::VoxelGrid<PointT>::filter_name_;
      using pcl::VoxelGrid<PointT>::getClassName;
      using pcl::VoxelGrid<PointT>::input_;
      using pcl::VoxelGrid<PointT>::indices_;
      using pcl::VoxelGrid<PointT>::filter_limit_negative_;
      using pcl::VoxelGrid<PointT>::filter_limit_min_;
      using pcl::VoxelGrid<PointT>::filter_limit_max_;
      using pcl::VoxelGrid<PointT>::filter_field_name_;

      using pcl::VoxelGrid<PointT>::downsample_all_data_;
      using pcl::VoxelGrid<PointT>::leaf_layout_;
      using pcl::VoxelGrid<PointT>::save_leaf_layout_;
      using pcl::VoxelGrid<PointT>::leaf_size_;
      using pcl::VoxelGrid<PointT>::min_b_;
      using pcl::VoxelGrid<PointT>::max_b_;
      using pcl::VoxelGrid<PointT>::inverse_leaf_size_;
      using pcl::VoxelGrid<PointT>::div_b_;
      using pcl::VoxelGrid<PointT>::divb_mul_;


      using FieldList = typename pcl::traits::fieldList<PointT>::type;
      using PointCloud = typename pcl::Filter<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

    public:

      using Ptr = pcl::shared_ptr<pcl::VoxelGrid<PointT> >;
      using ConstPtr = pcl::shared_ptr<const pcl::VoxelGrid<PointT> >;


      /** \brief Simple structure to hold a centroid, covarince and the number of points in a leaf.
        * Inverse covariance, eigen vectors and engen values are precomputed. */
      struct Leaf
      {
        /** \brief Constructor.
         * Sets \ref nr_points, \ref icov_, \ref mean_ and \ref evals_ to 0 and \ref cov_ and \ref evecs_ to the identity matrix
         */
        Leaf () :
          nr_points (0),
          Occupancy_probability_ (0.5),
          mean_ (Eigen::Vector3d::Zero ()),
          cov_ (Eigen::Matrix3d::Zero ()),
          icov_ (Eigen::Matrix3d::Zero ()),
          evecs_ (Eigen::Matrix3d::Identity ()),
          evals_ (Eigen::Vector3d::Zero ())
        {
        }

        /** \brief Get the voxel covariance.
          * \return covariance matrix
          */
        Eigen::Matrix3d
        getCov () const
        {
          return (cov_);
        }

        /** \brief Get the inverse of the voxel covariance.
         * \return inverse covariance matrix
         */
        Eigen::Matrix3d
        getInverseCov () const
        {
          return (icov_);
        }

        /** \brief Get the voxel centroid.
         * \return centroid
         */
        Eigen::Vector3d
        getMean () const
        {
          return (mean_);
        }

        /** \brief Get the eigen vectors of the voxel covariance.
         * \note Order corresponds with \ref getEvals
         * \return matrix whose columns contain eigen vectors
         */
        Eigen::Matrix3d
        getEvecs () const
        {
          return (evecs_);
        }

        /** \brief Get the eigen values of the voxel covariance.
         * \note Order corresponds with \ref getEvecs
         * \return vector of eigen values
         */
        Eigen::Vector3d
        getEvals () const
        {
          return (evals_);
        }

        /** \brief Get the number of points contained by this voxel.
         * \return number of points
         */
        int
        getPointCount () const
        {
          return (nr_points);
        }

        /******************************改动部分*************************************/
        /** \brief 获得本voxel的Oc值.
         * \return Oc
         */
        double
        getOc () const
        {
          return (Occupancy_probability_);
        }

        /** \brief 获得本voxel的均值的向量.
         * \return vector_of_mean_
         */
        const std::vector<Eigen::Vector3d>
        getVector_of_mean () const
        {
          return (vector_of_mean_);
        }
        /** \brief 获得本voxel的均值的方差.
         * \return vector_of_cov_
         */
        const std::vector<Eigen::Matrix3d>
        getVector_of_cov () const
        {
          return (vector_of_cov_);
        }
        /** \brief 获得本voxel的单帧点数的向量.
         * \return vector_of_nr_points_
         */
        const std::vector<int>
        getVector_of_nr_points () const
        {
          return (vector_of_nr_points_);
        }
        /** \brief 获得本voxel的单帧点云的向量.
         * \return vector_of_pc
         */
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
        getVector_of_pc() const {
          return (vector_of_pc);
        }
        /** \brief 设置本voxel的Oc值.
          * \return Oc
          */
        void
        setOc (double Oc)
        {
          Occupancy_probability_ = Oc;
        }

        /** \brief Number of points contained by voxel */
        int nr_points;

        /** \brief 3D voxel centroid */
        Eigen::Vector3d mean_;

        /** \brief Nd voxel centroid
         * \note Differs from \ref mean_ when color data is used
         */
        Eigen::VectorXf centroid;

        /** \brief Voxel covariance matrix */
        Eigen::Matrix3d cov_;

        /** \brief Inverse of voxel covariance matrix */
        Eigen::Matrix3d icov_;

        /** \brief Eigen vectors of voxel covariance matrix */
        Eigen::Matrix3d evecs_;

        /** \brief Eigen values of voxel covariance matrix */
        Eigen::Vector3d evals_;
        /******************************改动部分*************************************/
        /** \brief 体素的概率值 */
        double Occupancy_probability_;

        /** \brief 本体素内所包含的点云值 */
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vector_of_pc;

        /** \brief 表示这个格中每一帧的单帧均值 */
        std::vector<Eigen::Vector3d> vector_of_mean_;
        /** \brief 表示这个格中每一帧的单帧方差 */
        std::vector<Eigen::Matrix3d> vector_of_cov_;
        /** \brief 表示这个格中每一帧的点数 */
        std::vector<int> vector_of_nr_points_;
      };

      /** \brief Pointer to VoxelGridCovariance leaf structure */
      using LeafPtr = Leaf *;

      /** \brief Const pointer to VoxelGridCovariance leaf structure */
      using LeafConstPtr = const Leaf *;

    public:

      /** \brief Constructor.
       * Sets \ref leaf_size_ to 0 and \ref searchable_ to false.
       */
      VoxelGridCovariance () :
        searchable_ (true),
        min_points_per_voxel_ (6),
        min_covar_eigvalue_mult_ (0.01),
        leaves_ (),
        voxel_centroids_ (),
        kdtree_ (),
        voxel_leaf_centroids_indices_()
      {
        downsample_all_data_ = false;
        save_leaf_layout_ = false;
        leaf_size_.setZero ();
        min_b_.setZero ();
        max_b_.setZero ();
        filter_name_ = "VoxelGridCovariance";
      }

      /** \brief Set the minimum number of points required for a cell to be used (must be 3 or greater for covariance calculation).
        * \param[in] min_points_per_voxel the minimum number of points for required for a voxel to be used
        */
      inline void
      setMinPointPerVoxel (int min_points_per_voxel)
      {
        if(min_points_per_voxel > 2)
        {
          min_points_per_voxel_ = min_points_per_voxel;
        }
        else
        {
          PCL_WARN ("%s: Covariance calculation requires at least 3 points, setting Min Point per Voxel to 3 ", this->getClassName ().c_str ());
          min_points_per_voxel_ = 3;
        }
      }

      /** \brief Get the minimum number of points required for a cell to be used.
        * \return the minimum number of points for required for a voxel to be used
        */
      inline int
      getMinPointPerVoxel ()
      {
        return min_points_per_voxel_;
      }

      /** \brief Set the minimum allowable ratio between eigenvalues to prevent singular covariance matrices.
        * \param[in] min_covar_eigvalue_mult the minimum allowable ratio between eigenvalues
        */
      inline void
      setCovEigValueInflationRatio (double min_covar_eigvalue_mult)
      {
        min_covar_eigvalue_mult_ = min_covar_eigvalue_mult;
      }

      /** \brief Get the minimum allowable ratio between eigenvalues to prevent singular covariance matrices.
        * \return the minimum allowable ratio between eigenvalues
        */
      inline double
      getCovEigValueInflationRatio ()
      {
        return min_covar_eigvalue_mult_;
      }

      /** \brief Filter cloud and initializes voxel structure.
       * \param[out] output cloud containing centroids of voxels containing a sufficient number of points
       * \param[in] searchable flag if voxel structure is searchable, if true then kdtree is built
       */
      inline void
      filter (PointCloud &output, bool searchable = false)
      {
        searchable_ = searchable;
        applyFilter (output);

        voxel_centroids_ = PointCloudPtr (new PointCloud (output));

        if (searchable_ && !voxel_centroids_->empty ())
        {
          // Initiates kdtree of the centroids of voxels containing a sufficient number of points
          kdtree_.setInputCloud (voxel_centroids_);
        }
      }

      /** \brief Initializes voxel structure.
       * \param[in] searchable flag if voxel structure is searchable, if true then kdtree is built
       */
      inline void
      filter (bool searchable = false)
      {
        searchable_ = searchable;
        voxel_centroids_ = PointCloudPtr (new PointCloud);
        applyFilter (*voxel_centroids_);

        if (searchable_ && !voxel_centroids_->empty ())
        {
          // Initiates kdtree of the centroids of voxels containing a sufficient number of points
          kdtree_.setInputCloud (voxel_centroids_);
        }
      }


      /** \brief Get the voxel containing point p.
       * \param[in] index the index of the leaf structure node
       * \return const pointer to leaf structure
       */
      inline LeafConstPtr
      getLeaf (int index)
      {
        typename std::map<std::size_t, Leaf>::iterator leaf_iter = leaves_.find (index);
        if (leaf_iter != leaves_.end ())
        {
          LeafConstPtr ret (&(leaf_iter->second));
          return ret;
        }
        return nullptr;
      }

      /** \brief Get the voxel containing point p.
       * \param[in] p the point to get the leaf structure at
       * \return const pointer to leaf structure
       */
      inline LeafConstPtr
      getLeaf (PointT &p)
      {
        // Generate index associated with p
        int ijk0 = static_cast<int> (std::floor (p.x * inverse_leaf_size_[0]) - min_b_[0]);
        int ijk1 = static_cast<int> (std::floor (p.y * inverse_leaf_size_[1]) - min_b_[1]);
        int ijk2 = static_cast<int> (std::floor (p.z * inverse_leaf_size_[2]) - min_b_[2]);

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

        // Find leaf associated with index
        typename std::map<std::size_t, Leaf>::iterator leaf_iter = leaves_.find (idx);
        if (leaf_iter != leaves_.end ())
        {
          // If such a leaf exists return the pointer to the leaf structure
          LeafConstPtr ret (&(leaf_iter->second));
          return ret;
        }
        return nullptr;
      }

      /** \brief Get the voxel containing point p.
       * \param[in] p the point to get the leaf structure at
       * \return const pointer to leaf structure
       */
      inline LeafConstPtr
      getLeaf (Eigen::Vector3f &p)
      {
        // Generate index associated with p
        int ijk0 = static_cast<int> (std::floor (p[0] * inverse_leaf_size_[0]) - min_b_[0]);
        int ijk1 = static_cast<int> (std::floor (p[1] * inverse_leaf_size_[1]) - min_b_[1]);
        int ijk2 = static_cast<int> (std::floor (p[2] * inverse_leaf_size_[2]) - min_b_[2]);

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

        // Find leaf associated with index
        typename std::map<std::size_t, Leaf>::iterator leaf_iter = leaves_.find (idx);
        if (leaf_iter != leaves_.end ())
        {
          // If such a leaf exists return the pointer to the leaf structure
          LeafConstPtr ret (&(leaf_iter->second));
          return ret;
        }
        return nullptr;

      }

      /** \brief Get the voxels surrounding point p designated by #relative_coordinates.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] relative_coordinates 3xN matrix that represents relative coordinates of N neighboring voxels with respect to the center voxel
       * \param[in] reference_point the point to get the leaf structure at
       * \param[out] neighbors
       * \return number of neighbors found
       */
      int
      getNeighborhoodAtPoint (const Eigen::Matrix<int, 3, Eigen::Dynamic>& relative_coordinates, const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const;

      /** \brief Get the voxels surrounding point p, not including the voxel containing point p.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] reference_point the point to get the leaf structure at
       * \param[out] neighbors
       * \return number of neighbors found (up to 26)
       */
      int
      getNeighborhoodAtPoint (const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const;

      /** \brief Get the voxel at p.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] reference_point the point to get the leaf structure at
       * \param[out] neighbors
       * \return number of neighbors found (up to 1)
       */
      int
      getVoxelAtPoint (const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const;

      /** \brief Get the voxel at p and its facing voxels (up to 7 voxels).
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] reference_point the point to get the leaf structure at
       * \param[out] neighbors
       * \return number of neighbors found (up to 7)
       */
      int
      getFaceNeighborsAtPoint (const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const;

      /** \brief Get all 3x3x3 neighbor voxels of p (up to 27 voxels).
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] reference_point the point to get the leaf structure at
       * \param[out] neighbors
       * \return number of neighbors found (up to 27)
       */
      int
      getAllNeighborsAtPoint (const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const;

      /** \brief Get the leaf structure map
       * \return a map contataining all leaves
       */
      inline const std::map<std::size_t, Leaf>&
      getLeaves ()
      {
        return leaves_;
      }

      /** \brief Get a pointcloud containing the voxel centroids
       * \note Only voxels containing a sufficient number of points are used.
       * \return a map contataining all leaves
       */
      inline PointCloudPtr
      getCentroids ()
      {
        return voxel_centroids_;
      }


      /** \brief Get a cloud to visualize each voxels normal distribution.
       * \param[out] cell_cloud a cloud created by sampling the normal distributions of each voxel
       */
      void
      getDisplayCloud (pcl::PointCloud<pcl::PointXYZ>& cell_cloud);

      /** \brief Search for the k-nearest occupied voxels for the given query point.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] point the given query point
       * \param[in] k the number of neighbors to search for
       * \param[out] k_leaves the resultant leaves of the neighboring points
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
       * \return number of neighbors found
       */
      int
      nearestKSearch (const PointT &point, int k,
                      std::vector<LeafConstPtr> &k_leaves, std::vector<float> &k_sqr_distances) const
      {
        k_leaves.clear ();

        // Check if kdtree has been built
        if (!searchable_)
        {
          PCL_WARN ("%s: Not Searchable", this->getClassName ().c_str ());
          return 0;
        }

        // Find k-nearest neighbors in the occupied voxel centroid cloud
        pcl::Indices k_indices;
        k = kdtree_.nearestKSearch (point, k, k_indices, k_sqr_distances);

        // Find leaves corresponding to neighbors
        k_leaves.reserve (k);
        for (const auto &k_index : k_indices)
        {
          auto voxel = leaves_.find(voxel_centroids_leaf_indices_[k_index]);
          if (voxel == leaves_.end()) {
            continue;
          }

          k_leaves.push_back(&voxel->second);
        }
        return k_leaves.size();
      }

      /** \brief Search for the k-nearest occupied voxels for the given query point.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] cloud the given query point
       * \param[in] index the index
       * \param[in] k the number of neighbors to search for
       * \param[out] k_leaves the resultant leaves of the neighboring points
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
       * \return number of neighbors found
       */
      inline int
      nearestKSearch (const PointCloud &cloud, int index, int k,
                      std::vector<LeafConstPtr> &k_leaves, std::vector<float> &k_sqr_distances) const
      {
        if (index >= static_cast<int> (cloud.size ()) || index < 0)
          return (0);
        return (nearestKSearch (cloud[index], k, k_leaves, k_sqr_distances));
      }


      /** \brief Search for all the nearest occupied voxels of the query point in a given radius.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] point the given query point
       * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
       * \param[out] k_leaves the resultant leaves of the neighboring points
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
       * \param[in] max_nn
       * \return number of neighbors found
       */
      int
      radiusSearch (const PointT &point, double radius, std::vector<LeafConstPtr> &k_leaves,
                    std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
      {
        k_leaves.clear ();

        // Check if kdtree has been built
        if (!searchable_)
        {
          PCL_WARN ("%s: Not Searchable", this->getClassName ().c_str ());
          return 0;
        }

        // Find neighbors within radius in the occupied voxel centroid cloud
        pcl::Indices k_indices;
        const int k = kdtree_.radiusSearch (point, radius, k_indices, k_sqr_distances, max_nn);
        // Find leaves corresponding to neighbors
        k_leaves.reserve (k);
        for (const auto &k_index : k_indices)
        {
          const auto voxel = leaves_.find(voxel_centroids_leaf_indices_[k_index]);
          if(voxel == leaves_.end()) {
            continue;
          }

          k_leaves.push_back(&voxel->second);
        }
        return k_leaves.size();
      }

      /** \brief Search for all the nearest occupied voxels of the query point in a given radius.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] cloud the given query point
       * \param[in] index a valid index in cloud representing a valid (i.e., finite) query point
       * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
       * \param[out] k_leaves the resultant leaves of the neighboring points
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
       * \param[in] max_nn
       * \return number of neighbors found
       */
      inline int
      radiusSearch (const PointCloud &cloud, int index, double radius,
                    std::vector<LeafConstPtr> &k_leaves, std::vector<float> &k_sqr_distances,
                    unsigned int max_nn = 0) const
      {
        if (index >= static_cast<int> (cloud.size ()) || index < 0)
          return (0);
        return (radiusSearch (cloud[index], radius, k_leaves, k_sqr_distances, max_nn));
      }

      /*******************************************************改动部分********************************************************/

      /** \brief 在更新坐标时为每个坐标计算新的坐标值
       * \param[in] leaf的idex序号
       * \return leaf的中心坐标
       */
      inline
      const Eigen::Array4d GetLeafCenter(const int index) {
        int index_temp = index;

        //计算ijk
        Eigen::Vector4i ijk;
        ijk[2]=index_temp/divb_mul_[2];
        index_temp-=(ijk[2]*divb_mul_[2]);
        ijk[1]=index_temp/divb_mul_[1];
        index_temp-=(ijk[1]*divb_mul_[1]);
        ijk[0]=index_temp;
        ijk += min_b_;
        //计算新的index
        const Eigen::Array4d LeafCenter = ijk.array().template cast<double>() * leaf_size_.array().template cast<double>() +
                                          (leaf_size_ / 2).array().template cast<double>();
        return LeafCenter;
      }

      /** \brief 获得点云索引坐标向量
       * \note Only voxels containing a sufficient number of points are used.
       * \return a map contataining all leaves
       */
      inline std::vector<int>
      getVoxel_centroids_leaf_indices_ ()
      {
        return voxel_centroids_leaf_indices_;
      }

      /** \brief 更新每个voxel中的Oc值
       * \param[in] 传入计算得到的最终位姿的逆，即世界坐标系到雷达坐标系的转换位姿
       */
      void
      OcUpdate (const Eigen::Matrix4f &transform);
      /** \brief 对目标点云进行处理并更新voxel的值
       * \param[out] output cloud containing centroids of voxels containing a sufficient number of points
       */
      void
      applyUpdate (PointCloud &output);

     /** \brief 用新一帧点云去更新leaves_
      * \param[out] output cloud containing centroids of voxels containing a sufficient number of points
      */
      void
      UpdateLeaves (PointCloud &output);
     /** \brief 将一个点加入到新的ndt地图中去
       * \param[out] output cloud containing centroids of voxels containing a sufficient number of points
       */
      void
      addPointToLeavesCurr (const PointT &point, std::map<std::size_t, Leaf> &leaves_curr, int centroid_size);
     /** \brief 将新ndt地图Leaves_Curr更新到内部ndt地图Leaves中
       * \param[out] output cloud containing centroids of voxels containing a sufficient number of points
       */
      void
      addLeavesCurrToLeaves (const std::map<std::size_t, Leaf> &leaves_curr,PointCloud &output, int centroid_size, int rgba_index);
      /** \brief 更新 voxel structure.*/
      inline void
      upadate ()
      {
          searchable_ = true;
          applyUpdate (*voxel_centroids_);

          if (searchable_ && !voxel_centroids_->empty ())
          {
              // Initiates kdtree of the centroids of voxels containing a sufficient number of points
              kdtree_.setInputCloud (voxel_centroids_);
          }
      }
    /** \brief 获得概率更新后的点云
     */
    void
    getDistplayOcPointCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &displaypointcloud);
    /** \brief 获得概率更新后的点云
     */
    void
    getDistplaySingleOcPointCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &display_single_pointcloud);

      /*******************************************************改动部分********************************************************/
    protected:

      /*******************************************************改动部分********************************************************/

      /** \brief 在更新坐标时为每个坐标计算新的坐标值
       * \param[in] 老坐标值
       * \param[in] 新divb_mul_乘数值
       * \param[in] 新原点坐标值
       * \return 新坐标值
       */
      inline int
      Compute_New_index(const int index,
                        const Eigen::Vector4i& divb_mul_new_,
                        const Eigen::Vector4i& min_b_power_) {
        int new_index,index_temp;
        index_temp = index;

        //计算ijk
        Eigen::Vector4i ijk;
        ijk[2]=index_temp/divb_mul_[2];
        index_temp-=(ijk[2]*divb_mul_[2]);
        ijk[1]=index_temp/divb_mul_[1];
        index_temp-=(ijk[1]*divb_mul_[1]);
        ijk[0]=index_temp;
        ijk+=min_b_;

        //计算新的index
        new_index=(ijk-min_b_power_).dot(divb_mul_new_);
        return new_index;
      }
      /*******************************************************改动部分********************************************************/

      /** \brief Filter cloud and initializes voxel structure.
       * \param[out] output cloud containing centroids of voxels containing a sufficient number of points
       */
      void applyFilter (PointCloud &output) override;

      /** \brief Flag to determine if voxel structure is searchable. */
      bool searchable_;

      /** \brief Minimum points contained with in a voxel to allow it to be usable. */
      int min_points_per_voxel_;

      /** \brief Minimum allowable ratio between eigenvalues to prevent singular covariance matrices. */
      double min_covar_eigvalue_mult_;

      /** \brief Voxel structure containing all leaf nodes (includes voxels with less than a sufficient number of points). */
      std::map<std::size_t, Leaf> leaves_;

      /** \brief Point cloud containing centroids of voxels containing atleast minimum number of points. */
      PointCloudPtr voxel_centroids_;

      /** \brief Indices of leaf structurs associated with each point in \ref voxel_centroids_ (used for searching). */
      std::vector<int> voxel_centroids_leaf_indices_;

      /** \brief KdTree generated using \ref voxel_centroids_ (used for searching). */
      pcl::KdTreeFLANN<PointT> kdtree_;
      /*******************************************************改动部分********************************************************/
      /** \brief 表示从体素的idx到对应重心点云中点的序号的映射. */
      std::map<int, int> voxel_leaf_centroids_indices_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/voxel_grid_covariance.hpp>
#endif
