#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>


using namespace std::chrono_literals;

class PcTestNode : public rclcpp::Node {
public:
    PcTestNode() : Node("pc_test_node") {

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("raw_pc2", 10);

        auto publish_msg_callback = [this]() -> void {
            auto message = sensor_msgs::msg::PointCloud2();
            //create a point cloud message
            message.header.frame_id = "map";
            message.height = 1;
            message.width = 4;
            message.fields.resize(3);
            message.fields[0].name = "x";
            message.fields[0].offset = 0;
            message.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            message.fields[0].count = 1;
            message.fields[1].name = "y";
            message.fields[1].offset = 4;
            message.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            message.fields[1].count = 1;
            message.fields[2].name = "z";
            message.fields[2].offset = 8;
            message.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            message.fields[2].count = 1;
            message.is_bigendian = false;
            message.point_step = 12;
            message.row_step = 12;
            message.is_dense = true;
            message.data.resize(message.point_step * message.width);
            float *data = reinterpret_cast<float *>(message.data.data());
            data[0] = 2.0;
            data[1] = 2.0;
            data[2] = 0.0;
            data[3] = 1.0;
            data[4] = 1.0;
            data[5] = 0.0;

            this->publisher_->publish(message);
        }; 
        timer_ = this->create_wall_timer(500ms, publish_msg_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcTestNode>());
    rclcpp::shutdown();
    return 0;
}