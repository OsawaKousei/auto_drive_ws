#pragma once

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

namespace viz_marker{

static visualization_msgs::msg::Marker::SharedPtr std_cube_setter(std::tuple<double, double> position){
  auto message = visualization_msgs::msg::Marker();

  message.type = visualization_msgs::msg::Marker::CUBE;
  message.action = visualization_msgs::msg::Marker::ADD;
  message.lifetime = rclcpp::Duration(0,0);

  message.scale.x = 0.1;
  message.scale.y = 0.1;
  message.scale.z = 0.01;

  message.pose.position.x = std::get<0>(position);
  message.pose.position.y = std::get<1>(position);
  message.pose.position.z = 0.0;

  message.pose.orientation.x = 0.0;
  message.pose.orientation.y = 0.0;
  message.pose.orientation.z = 0.0;
  message.pose.orientation.w = 1.0;

  message.color.r = 0.0f;
  message.color.g = 1.0f;
  message.color.b = 0.0f;
  message.color.a = 1.0;

  return std::make_shared<visualization_msgs::msg::Marker>(message);
}

static visualization_msgs::msg::Marker::SharedPtr std_line_setter(std::tuple<double, double> start_pos, std::tuple<double, double> end_pos){
  visualization_msgs::msg::Marker message;

  message.type = visualization_msgs::msg::Marker::LINE_STRIP;
  message.action = visualization_msgs::msg::Marker::ADD;
  message.lifetime = rclcpp::Duration(0,0);

  message.scale.x = 0.01;

  message.color.r = 0.0f;
  message.color.g = 0.0f;
  message.color.b = 1.0f;
  message.color.a = 1.0;

  geometry_msgs::msg::Point start_point;
  start_point.x = std::get<0>(start_pos);
  start_point.y = std::get<1>(start_pos);
  start_point.z = 0.0;

  geometry_msgs::msg::Point end_point;
  end_point.x = std::get<0>(end_pos);
  end_point.y = std::get<1>(end_pos);

  message.points.push_back(start_point);
  message.points.push_back(end_point);

  std::cout << "create line from (" << start_point.x << ", " << start_point.y << ") to (" << end_point.x << ", " << end_point.y << ")" << std::endl;

  return std::make_shared<visualization_msgs::msg::Marker>(message);
}

};