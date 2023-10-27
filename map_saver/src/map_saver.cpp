#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class MapSaverNode : public rclcpp::Node {
public:
    MapSaverNode() : Node("map_saver"), point_cloud_seq(0) {
        pose_subscription = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/slam/pose", 10, std::bind(&MapSaverNode::pose_callback, this, std::placeholders::_1));

        point_cloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
            "point_cloud", rclcpp::SensorDataQoS());

        // Initialize point cloud
        point_cloud.header.frame_id = "map";
        point_cloud.is_dense = true;
        point_cloud.width = 0;
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        float x = msg->pose.position.x;
        float y = msg->pose.position.y;
        float z = msg->pose.position.z;

        point_cloud_seq++;

        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        point_cloud.push_back(point);
        point_cloud.width++;

        RCLCPP_INFO(this->get_logger(), "Received pose: x=%f, y=%f, z=%f", x, y, z);

        if (point_cloud_seq % 100 == 0) {
            // Convert PCL point cloud to sensor_msgs::msg::PointCloud2
            sensor_msgs::msg::PointCloud2 ros_cloud;
            pcl::toROSMsg(point_cloud, ros_cloud);
            ros_cloud.header.stamp = rclcpp::Time();
            ros_cloud.header.frame_id = "map";

            point_cloud_publisher->publish(ros_cloud);

            pcl::io::savePCDFileBinary("point_cloud.pcd", point_cloud);

            RCLCPP_INFO(this->get_logger(), "Saved 3D point cloud as point_cloud.pcd");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    uint32_t point_cloud_seq;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
