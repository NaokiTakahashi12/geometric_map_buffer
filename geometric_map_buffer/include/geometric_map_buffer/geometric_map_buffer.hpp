// MIT License
//
// Copyright (c) 2023 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_srvs/srv/empty.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace geometric_map_buffer
{
class GeometricMapBuffer
{
public:
  using UniquePtr = std::unique_ptr<GeometricMapBuffer>;
  using GridMapUniquePtr = std::unique_ptr<grid_map::GridMap>;

  explicit GeometricMapBuffer(
    rclcpp::Node &,
    const std::string & base_topic = "~");
  ~GeometricMapBuffer();

  void enablePublishGridMapService(rclcpp::Node &);

  void publishGridMap();

  void publishInitialGridMap();
  void publishInitialGridMap(GridMapUniquePtr);

  void setGridMap(GridMapUniquePtr);

  GridMapUniquePtr getGridMap();
  grid_map::GridMap & accessGridMap();
  GridMapUniquePtr getGridSubmap(
    const grid_map::Position &, const grid_map::Length &
  );

private:
  std::string m_base_topic;

  std::mutex m_grid_map_mutex;

  GridMapUniquePtr m_grid_map_buffer;

  rclcpp::Clock::SharedPtr m_node_clock;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
  //! base_topic/grid_map
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr m_grid_map_publisher;
  //! base_topic/init_grid_map
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr m_initial_grid_map_publisher;
  //! base_topic/init_grid_map
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr m_init_grid_map_subscription;
  //! base_topic/grid_submap
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr m_grid_submap_subscription;
  //! base_topic/publish_grid_map
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_publish_grid_map_service;

  void initGridMapCallback(grid_map_msgs::msg::GridMap::ConstSharedPtr);
  void gridSubmapCallback(grid_map_msgs::msg::GridMap::ConstSharedPtr);
  void publishGridMapService(
    std_srvs::srv::Empty::Request::ConstSharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr
  );
};
}  // namespace geometric_map_buffer
