
void
ComputeProbability::point_projection(pcl::PointXYZ p, int &col, int &ring_id) {
  double _factor = ((_rings - 1) / (_upperBound - _lowerBound));
  double col_factor = 360.0 / _cols;
  float theta = 0;
  if (p.y == 0 && p.x == 0) theta = 0;
  else if (p.x >= 0) {
    float tan_theta = -p.y / p.x;
    theta = 180 * std::atan(tan_theta) / M_PI + 90.00001;
  }
  else {
    float tan_theta = -p.y / p.x;
    theta = 180 * std::atan(tan_theta) / M_PI + 270;
  }
  col = cvFloor(theta / col_factor); // theta [0, 180] ==> [0, 1000]
  float hypotenuse = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));
  float angle = std::atan(p.z / hypotenuse);
  ring_id = int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}


void
ComputeProbability::pointCloud_To_img() {
  std::vector<std::vector<float> > pc_image(_rings,std::vector<float>(_cols,-1));
  // convert pointcloud from 3D to 2D img
  for (size_t i=0 ; i<_pc->size() ; i++) {
    int col,ring_id;
    point_projection(_pc->points[i], col, ring_id);

    if (ring_id < 0 ||
        ring_id > _rings-1) continue;
    float dist = std::sqrt(
            std::pow(_pc->points[i].y, 2) +
            std::pow(_pc->points[i].x, 2) +
            std::pow(_pc->points[i].z, 2)
    );
    if(dist < 0.1) continue; //10
    if(pc_image[ring_id][col] == -1) {
      pc_image[ring_id][col] = dist;//range
    }
    else if(dist < pc_image[ring_id][col]) {
      pc_image[ring_id][col] = dist;//set the nearer point
    }
  }
  for (unsigned int i = 0; i < pc_image.size(); ++i)
  {
    cv::Mat Sample(1, _cols, cv::DataType<float>::type, pc_image[i].data());
    _image.push_back(Sample);
  }
}

double
ComputeProbability::compute_PointProbability(pcl::PointXYZ point) {
  int col,ring_id;
  point_projection(point, col, ring_id);
  if (ring_id < 0) return 0.7;
  if (ring_id > _rings-1) return 0.5;

  float dist = std::sqrt(
          std::pow(point.y, 2) +
          std::pow(point.x, 2) +
          std::pow(point.z, 2)
  );
  //if(dist < 2) return 0.5;
  double p = p_r_z(dist , _image.at<float>(ring_id, col));
  return p;
}