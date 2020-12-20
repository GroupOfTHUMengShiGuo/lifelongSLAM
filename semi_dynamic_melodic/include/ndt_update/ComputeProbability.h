//
// Created by zxd on 20-12-2.
//
#ifndef SRC_COMPUTEPROBABILITY_H
#define SRC_COMPUTEPROBABILITY_H
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <opencv2/opencv.hpp>
class ComputeProbability{
public:
  explicit ComputeProbability(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc)
      : _sigma(0.5),
        _width(1),
        _upperBound(2),
        _lowerBound(-24.8),
        _rings(256),
        _cols(1800),
        _pc(pc),
        _image(cv::Mat(0, _cols, cv::DataType<float>::type)) {
    pointCloud_To_img();
  }

  /** \brief ComputeProbability的构造函数，对于成员变量进行赋值
    * \param[in] 体素的边长
    */
  explicit ComputeProbability(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc, double width)
      : _sigma(0.5),
        _width(width),
        _upperBound(2),
        _lowerBound(-24.8),
        _rings(256),
        _cols(1800),
        _pc(pc),
        _image(cv::Mat(0, _cols, cv::DataType<float>::type)) {
    pointCloud_To_img();
  }
  /** \brief 初始化成员变量
    * \param[in] 体素的边长
    */
  // TODO
  void
  init () {}


  /** \brief 得到当前帧的深度图
    * \return 当前帧的深度图
    */
  inline cv::Mat
  getImg() {return _image;}

  /** \brief 通过论文Generic NDT mapping in dynamic environments and its application for lifelong SLAM的公式(7)计算概率
    * \param[in] 公式7中的r
    * \param[in] 公式7中的z
    * \return 计算得到的概率值
    */
  inline double
  p_r_z (const double r,
         const double z) const {
    double x1 = (r - z) / _width + 0.5, x2 = (r - z) / _width - 0.5, s = _sigma / _width;
    double ln2 = log(2.0);
    double p = (0.5 + 0.5 * erf((s * s * ln2 + x1) / (sqrt(2.0) * s))) -
               0.5 * (0.5 + 0.5 * erf((s * s * ln2 + x2) / (sqrt(2.0) * s)));
    return p;
  }
  /** \brief 计算三维坐标点投影到二维柱面的角度值
    * \param[in] 要进行投影的点
    * \param[out] 计算出的投影的横坐标
    * \param[out] 计算出的投影的纵坐标
    */
  void
  point_projection(pcl::PointXYZ p, int& col, int& ring_id);

  /** \brief 将三维点云投影到柱面成为一个图片，图片的值为当前像素方向上对应的距离
    */
  void
  pointCloud_To_img();

  /** \brief 计算某个点在当前帧下的概率值
  * \param[in] 要计算的点
  * \return 得到的概率值
  */
  double
  compute_PointProbability(pcl::PointXYZ point);
private:
  /** \brief 激光雷达测量的误差的方差 */
  const double _sigma;

  /** \brief 体素的宽度 */
  double _width;

  /** \brief 激光雷达最高的角度 */
  double _upperBound;

  /** \brief 激光雷达最低的角度 */
  double _lowerBound;

  /** \brief 激光雷达的线数 */
  int _rings;

  /** \brief 激光点云投影到的图片的水平方向的分辨率 */
  int _cols;

  /** \brief 新来的一帧点云 */
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr _pc;

  /** \brief 新来的一帧点云所对应的深度图 */
  cv::Mat _image;
};

#include "ComputeProbability.hpp"
#endif //SRC_COMPUTEPROBABILITY_H
