#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>

# define M_PI 3.14159265358979323846

class MeasureGroundAngleNode : public rclcpp::Node {
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noground_pub;

public:
  MeasureGroundAngleNode()
    : Node("measure_ground_angle")
  {
    angle_pub = this->create_publisher<std_msgs::msg::Float64>("/ground_angle", 10);
    ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_cloud", 10);
    noground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/noground_cloud", 10);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", qos, std::bind(&MeasureGroundAngleNode::scanCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Running");
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ros_cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_cloud, *cloud);


    /* Find the ground plane using RANSAC */
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(3000);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud);
    // After running RANSAC, compute a new plane that best fits the original inliers
    // hopefully obtaining a more accurate result
    seg.setOptimizeCoefficients(true);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() <= 0) {
      RCLCPP_WARN(this->get_logger(), "No planes detected!");
      return;
    }


    /* Extract clouds from indices and publish as ROS messages */
    extract_indices_and_publish(cloud, inliers, ground_pub, false);
    extract_indices_and_publish(cloud, inliers, noground_pub, true);


    /* Compute the actual angle and publish it */

    // The plane normal is equal to the normalized coefficients
    Eigen::Vector3f plane_normal(
      coefficients->values[0],
      coefficients->values[1],
      coefficients->values[2]
    );
    plane_normal.normalize();
    
    // Scalar angle between up vector and plane normal (ground is supposed to be horizontal)
    float ang = std::acos(Eigen::Vector3f::UnitZ().dot(plane_normal)) * (180.0 / M_PI);

    angle_pub->publish(ang);
    RCLCPP_INFO(this->get_logger(), "%.2f degrees", ang);
  }

private:
  void extract_indices_and_publish(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
        const pcl::PointIndices::Ptr& indices,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
        bool negate)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    extract.setInputCloud(cloud_in);
    extract.setIndices(indices);
    extract.setNegative(negate);
    extract.filter(*cloud);

    sensor_msgs::msg::PointCloud2::SharedPtr msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*cloud, *msg);
    msg->header.frame_id = "velodyne";
    publisher->publish(*msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MeasureGroundAngleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}