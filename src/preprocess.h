#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE { AVIA = 1, VELO16, L515, OUSTER64, MID360, ROBOSENSE };

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(float, time, time)(uint16_t,
                                                                    ring, ring))

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint16_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros

namespace livox_pcl2 {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace livox_pcl2

namespace robosense_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace robosense_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_pcl2::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, tag, tag)
    (std::uint8_t, line, line)
    (double, timestamp, timestamp)
)
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;
  double blind;
  bool feature_enabled, given_offset_time;
    

  private:
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg);
  void oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void mid360_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void robosense_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  
};
