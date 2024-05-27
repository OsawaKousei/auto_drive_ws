#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class Scan2pc2Node : public rclcpp::Node {
public:
    Scan2pc2Node() : Node("scan2pc2_node") {
        pc2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("raw_pc2", 10);

        auto topic_callback = [this](const sensor_msgs::msg::LaserScan &msg) -> void {
            auto message = sensor_msgs::msg::PointCloud2();
            message = scan2pc2(msg);
            this->pc2_pub->publish(message);
        }; 

        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, topic_callback);
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub;

    sensor_msgs::msg::PointCloud2 scan2pc2(const sensor_msgs::msg::LaserScan &scan) {
        sensor_msgs::msg::PointCloud2 pc2;
        // convert LaserScan to PointCloud2
        pc2.header = scan.header;
        pc2.height = 1;
        pc2.width = scan.ranges.size();
        pc2.fields.resize(3);
        pc2.fields[0].name = "x";
        pc2.fields[0].offset = 0;
        pc2.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc2.fields[0].count = 1;
        pc2.fields[1].name = "y";
        pc2.fields[1].offset = 4;
        pc2.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc2.fields[1].count = 1;
        pc2.fields[2].name = "z";
        pc2.fields[2].offset = 8;
        pc2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc2.fields[2].count = 1;
        pc2.is_bigendian = false;
        pc2.point_step = 12;
        pc2.row_step = 12 * scan.ranges.size();
        pc2.is_dense = true;
        pc2.data.resize(pc2.point_step * pc2.width);
        float *data = reinterpret_cast<float *>(pc2.data.data());
        for (int i = 0; i < scan.ranges.size(); i++) {
            data[i * 3] = scan.ranges[i] * cos(scan.angle_min + i * scan.angle_increment);
            data[i * 3 + 1] = scan.ranges[i] * sin(scan.angle_min + i * scan.angle_increment);
            data[i * 3 + 2] = 0.0;
        }
        return pc2;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Scan2pc2Node>());
    rclcpp::shutdown();
    return 0;
}