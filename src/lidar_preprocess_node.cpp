#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

class LidarPreprocessNode : public rclcpp::Node {
public:
  LidarPreprocessNode() : Node("lidar_preprocess_node") {
    declare_parameter("min_distance", 0.5);
    declare_parameter("height_min", -2.0);
    declare_parameter("height_max", 5.0);
    declare_parameter("sor_mean_k", 50);
    declare_parameter("sor_stddev", 1.0);
    declare_parameter("enable_distance_filter", true);
    declare_parameter("enable_noise_filter", true);
    declare_parameter("enable_height_filter", true);
    declare_parameter("enable_footprint_filter", true);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input", 10, std::bind(&LidarPreprocessNode::callback, this, std::placeholders::_1));
    footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
        "footprint", 10, std::bind(&LidarPreprocessNode::footprintCallback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*msg, *cloud);

    if (get_parameter("enable_distance_filter").as_bool()) {
      cloud = filterDistance(cloud);
    }

    if (get_parameter("enable_height_filter").as_bool()) {
      cloud = filterHeight(cloud);
    }

    if (get_parameter("enable_noise_filter").as_bool()) {
      cloud = filterNoise(cloud);
    }

    if (get_parameter("enable_footprint_filter").as_bool() && footprint_) {
      cloud = filterFootprint(cloud);
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = msg->header;
    pub_->publish(output);
  }

  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    footprint_ = msg;
  }

  PointCloud::Ptr filterDistance(const PointCloud::Ptr& cloud) {
    PointCloud::Ptr filtered(new PointCloud);
    double min_dist_sq = get_parameter("min_distance").as_double();
    min_dist_sq *= min_dist_sq;

    for (const auto& point : cloud->points) {
      double dist_sq = point.x * point.x + point.y * point.y + point.z * point.z;
      if (dist_sq >= min_dist_sq) {
        filtered->push_back(point);
      }
    }
    return filtered;
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
    PointCloud::Ptr filtered(new PointCloud);

    std::vector<float> x_ary, y_ary;
    for (const auto& pt : footprint_->polygon.points) {
      x_ary.push_back(pt.x);
      y_ary.push_back(pt.y);
    }

    for (const auto& point : cloud->points) {
      float x = point.x;
      float y = point.y;

      // Ray casting algorithm
      size_t i, j;
      bool inside = false;
      for (i = 0, j = x_ary.size() - 1; i < x_ary.size(); j = i++) {
        if (((y_ary[i] > y) != (y_ary[j] > y)) &&
            (x < (x_ary[j] - x_ary[i]) * (y - y_ary[i]) / (y_ary[j] - y_ary[i]) + x_ary[i])) {
          inside = !inside;
        }
      }

      if (!inside) {
        filtered->push_back(point);
      }
    }
    return filtered;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
