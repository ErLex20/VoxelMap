#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1) {
  N_SCANS = 6;
  given_offset_time = false;
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num) {
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg,
                         PointCloudXYZI::Ptr &pcl_out) {
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                         PointCloudXYZI::Ptr &pcl_out) {
  switch (lidar_type) {
  case L515:
    l515_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case OUSTER64:
    oust64_handler(msg);
    break;

  case MID360:
    mid360_handler(msg);
    break;

  case ROBOSENSE:
    robosense_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  int plsize = msg->point_num;
  std::vector<bool> is_valid_pt(plsize, false);

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS; i++) {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  for (uint i = 1; i < plsize; i++) {
    if ((msg->points[i].line < N_SCANS) &&
        ((msg->points[i].tag & 0x30) == 0x10 ||
         (msg->points[i].tag & 0x30) == 0x00)) {
      valid_num++;
      if (i % point_filter_num == 0) {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature =
            msg->points[i].offset_time /
            float(1000000); // use curvature as time of each laser points

        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
            (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
            (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) &&
                (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y +
                     pl_full[i].z + pl_full[i].z >
                 blind * blind)) {
          is_valid_pt[i] = true;
        }
      }
    }
  }

  for (uint i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      pl_surf.points.push_back(pl_full[i]);
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  // cout << "===================================" << endl;
  // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
  for (int i = 0; i < pl_orig.points.size(); i++) {
    if (i % point_filter_num != 0)
      continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x +
                   pl_orig.points[i].y * pl_orig.points[i].y +
                   pl_orig.points[i].z * pl_orig.points[i].z;

    if (!std::isfinite(range) || range <= blind * blind)
      continue;

    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
    if (yaw_angle >= 180.0)
      yaw_angle -= 360.0;
    if (yaw_angle <= -180.0)
      yaw_angle += 360.0;

    added_pt.curvature = pl_orig.points[i].t / 1e6;

    pl_surf.points.push_back(added_pt);
  }
}

void Preprocess::l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  pl_surf.clear();
  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  // pl_surf.reserve(plsize);
  for (int i = 0; i < pl_orig.size(); i++) {
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    if (i % point_filter_num == 0) {
      pl_surf.push_back(added_pt);
    }
  }
}

#define MAX_LINE_NUM 64

void Preprocess::velodyne_handler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  if (pl_orig.empty()) {
    return;
  }

  pl_surf.reserve(pl_orig.size());
  const float first_time = pl_orig.front().time;
  for (std::size_t i = 0; i < pl_orig.size(); ++i) {
    if (i % point_filter_num != 0) {
      continue;
    }

    const auto &source = pl_orig[i];
    const double range_squared = source.x * source.x + source.y * source.y +
                                 source.z * source.z;
    if (!std::isfinite(range_squared) || range_squared <= blind * blind) {
      continue;
    }

    PointType added_pt;
    added_pt.x = source.x;
    added_pt.y = source.y;
    added_pt.z = source.z;
    added_pt.intensity = source.intensity;
    added_pt.normal_x = 0.0f;
    added_pt.normal_y = 0.0f;
    added_pt.normal_z = 0.0f;
    // IILABS3D VLP-16 times are relative seconds and may start near -0.1.
    added_pt.curvature = (source.time - first_time) * 1000.0f;
    pl_surf.push_back(added_pt);
  }
}

void Preprocess::mid360_handler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<livox_pcl2::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  if (pl_orig.empty()) {
    return;
  }

  pl_surf.reserve(pl_orig.size());
  const double first_timestamp = pl_orig.front().timestamp;
  for (std::size_t i = 0; i < pl_orig.size(); ++i) {
    if (i % point_filter_num != 0) {
      continue;
    }

    const auto &source = pl_orig[i];
    const double range_squared = source.x * source.x + source.y * source.y +
                                 source.z * source.z;
    if (range_squared <= blind * blind) {
      continue;
    }

    PointType point;
    point.x = source.x;
    point.y = source.y;
    point.z = source.z;
    point.intensity = source.intensity;
    point.normal_x = 0.0f;
    point.normal_y = 0.0f;
    point.normal_z = 0.0f;
    // IILABS3D stores absolute per-point timestamps as float64 nanoseconds.
    // VoxelMap uses curvature as the offset from scan start in milliseconds.
    point.curvature = static_cast<float>((source.timestamp - first_timestamp) * 1e-6);
    pl_surf.push_back(point);
  }
}

void Preprocess::robosense_handler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<robosense_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  if (pl_orig.empty()) {
    return;
  }

  pl_surf.reserve(pl_orig.size());
  const double first_timestamp = pl_orig.front().timestamp;
  for (std::size_t i = 0; i < pl_orig.size(); ++i) {
    if (i % point_filter_num != 0) {
      continue;
    }

    const auto &source = pl_orig[i];
    const double range_squared = source.x * source.x + source.y * source.y +
                                 source.z * source.z;
    if (!std::isfinite(range_squared) || range_squared <= blind * blind) {
      continue;
    }

    PointType point;
    point.x = source.x;
    point.y = source.y;
    point.z = source.z;
    point.intensity = source.intensity;
    point.normal_x = 0.0f;
    point.normal_y = 0.0f;
    point.normal_z = 0.0f;
    // RSLidar timestamps are absolute seconds; curvature stores milliseconds
    // from the beginning of the scan for IMU de-skewing.
    point.curvature = static_cast<float>((source.timestamp - first_timestamp) * 1000.0);
    pl_surf.push_back(point);
  }
}
