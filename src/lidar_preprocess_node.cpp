#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

class LidarPreprocessNode : public rclcpp::Node {
public:
  LidarPreprocessNode() : Node("lidar_preprocess_node") {
    declare_parameter("height_min", -2.0);
    declare_parameter("height_max", 5.0);
    declare_parameter("footprint_x", 1.5);
    declare_parameter("footprint_y", 1.0);
    declare_parameter("sor_mean_k", 50);
    declare_parameter("sor_stddev", 1.0);
    declare_parameter("enable_noise_filter", true);
    declare_parameter("enable_height_filter", true);
    declare_parameter("enable_footprint_filter", true);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input", 10, std::bind(&LidarPreprocessNode::callback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*msg, *cloud);

    if (get_parameter("enable_footprint_filter").as_bool()) {
      cloud = filterFootprint(cloud);
    }

    if (get_parameter("enable_height_filter").as_bool()) {
      cloud = filterHeight(cloud);
    }

    if (get_parameter("enable_noise_filter").as_bool()) {
      cloud = filterNoise(cloud);
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = msg->header;
    pub_->publish(output);
  }

  PointCloud::Ptr filterHeight(const PointCloud::Ptr& cloud) {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(get_parameter("height_min").as_double(),
                         get_parameter("height_max").as_double());
    PointCloud::Ptr filtered(new PointCloud);
    pass.filter(*filtered);
    return filtered;
  }

  PointCloud::Ptr filterNoise(const PointCloud::Ptr& cloud) {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(get_parameter("sor_mean_k").as_int());
    sor.setStddevMulThresh(get_parameter("sor_stddev").as_double());
    PointCloud::Ptr filtered(new PointCloud);
    sor.filter(*filtered);
    return filtered;
  }

  PointCloud::Ptr filterFootprint(const PointCloud::Ptr& cloud) {
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(cloud);
    double x = get_parameter("footprint_x").as_double();
    double y = get_parameter("footprint_y").as_double();
    crop.setMin(Eigen::Vector4f(-x, -y, -10.0, 1.0));
    crop.setMax(Eigen::Vector4f(x, y, 10.0, 1.0));
    crop.setNegative(true);
    PointCloud::Ptr filtered(new PointCloud);
    crop.filter(*filtered);
    return filtered;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
