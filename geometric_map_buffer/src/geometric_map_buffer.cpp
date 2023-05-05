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

#include <geometric_map_buffer/geometric_map_buffer.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace geometric_map_buffer
{
//! @todo Parameterize QoS buffer size
GeometricMapBuffer::GeometricMapBuffer(
  rclcpp::Node & node,
  const std::string & base_topic)
: m_grid_map_mutex(),
  m_grid_map_buffer(nullptr),
  m_node_clock(nullptr),
  m_tf_buffer(nullptr),
  m_tf_listener(nullptr),
  m_grid_map_publisher(nullptr),
  m_initial_grid_map_publisher(nullptr),
  m_grid_submap_subscription(nullptr)
{
  m_grid_map_buffer = std::make_unique<grid_map::GridMap>();

  m_node_clock = node.get_clock();
  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(m_node_clock);
  m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);

  m_grid_map_publisher = node.create_publisher<grid_map_msgs::msg::GridMap>(
    base_topic + "/grid_map",
    rclcpp::QoS(1)
  );
  m_initial_grid_map_publisher = node.create_publisher<grid_map_msgs::msg::GridMap>(
    base_topic + "/init_grid_map",
    rclcpp::QoS(1)
  );
  m_init_grid_map_subscription = node.create_subscription<grid_map_msgs::msg::GridMap>(
    base_topic + "/init_grid_map",
    rclcpp::QoS(1),
    std::bind(
      &GeometricMapBuffer::initGridMapCallback,
      this,
      std::placeholders::_1
    )
  );
  m_grid_submap_subscription = node.create_subscription<grid_map_msgs::msg::GridMap>(
    base_topic + "/grid_submap",
    rclcpp::QoS(2),
    std::bind(
      &GeometricMapBuffer::gridSubmapCallback,
      this,
      std::placeholders::_1));
  m_publish_grid_map_service = node.create_service<std_srvs::srv::Empty>(
    base_topic + "/publish_grid_map",
    std::bind(
      &GeometricMapBuffer::publishGridMapService,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
}

GeometricMapBuffer::~GeometricMapBuffer() {}

void GeometricMapBuffer::publishGridMap()
{
  if (not m_grid_map_publisher) {
    throw std::runtime_error("Failed publish grid map");
  }
  if (not m_grid_map_buffer) {
    throw std::runtime_error("Failed access grid map buffer");
  }
  std::lock_guard<std::mutex> grid_map_buffer_lock{m_grid_map_mutex};
  grid_map_msgs::msg::GridMap::UniquePtr grid_map_msg;
  grid_map_msg = grid_map::GridMapRosConverter::toMessage(*m_grid_map_buffer);
  m_grid_map_publisher->publish(std::move(grid_map_msg));
}

void GeometricMapBuffer::publishInitialGridMap()
{
  if (not m_grid_map_publisher) {
    throw std::runtime_error("Failed publish initial grid map");
  }
  if (not m_grid_map_buffer) {
    throw std::runtime_error("Failed access grid map buffer");
  }
  std::lock_guard<std::mutex> grid_map_buffer_lock{m_grid_map_mutex};
  grid_map_msgs::msg::GridMap::UniquePtr grid_map_msg;
  grid_map_msg = grid_map::GridMapRosConverter::toMessage(*m_grid_map_buffer);
  m_initial_grid_map_publisher->publish(std::move(grid_map_msg));
}

void GeometricMapBuffer::publishInitialGridMap(GridMapUniquePtr grid_map)
{
  if (not m_grid_map_publisher) {
    throw std::runtime_error("Failed publish initial grid_map");
  }
  if (not grid_map) {
    throw std::runtime_error("Failed access grid map");
  }
  grid_map_msgs::msg::GridMap::UniquePtr grid_map_msg;
  grid_map_msg = grid_map::GridMapRosConverter::toMessage(*grid_map);
  m_initial_grid_map_publisher->publish(std::move(grid_map_msg));
}

void GeometricMapBuffer::setGridMap(GeometricMapBuffer::GridMapUniquePtr new_grid_map)
{
  std::lock_guard<std::mutex> grid_map_buffer_lock{m_grid_map_mutex};
  m_grid_map_buffer.reset();
  m_grid_map_buffer = std::move(new_grid_map);
}

GeometricMapBuffer::GridMapUniquePtr GeometricMapBuffer::getGridMap()
{
  std::lock_guard<std::mutex> grid_map_buffer_lock{m_grid_map_mutex};
  return std::make_unique<grid_map::GridMap>(*m_grid_map_buffer);
}

grid_map::GridMap & GeometricMapBuffer::accessGridMap()
{
  std::lock_guard<std::mutex> grid_map_buffer_lock{m_grid_map_mutex};
  return *m_grid_map_buffer;
}

GeometricMapBuffer::GridMapUniquePtr GeometricMapBuffer::getGridSubmap(
  const grid_map::Position & position,
  const grid_map::Length & length)
{
  std::lock_guard<std::mutex> grid_map_buffer_lock{m_grid_map_mutex};
  bool is_success;
  return std::make_unique<grid_map::GridMap>(
    m_grid_map_buffer->getSubmap(position, length, is_success)
  );
}

void GeometricMapBuffer::initGridMapCallback(
  grid_map_msgs::msg::GridMap::ConstSharedPtr grid_map_msg)
{
  std::lock_guard<std::mutex> grid_map_buffer_lock{m_grid_map_mutex};
  grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, *m_grid_map_buffer);
}

void GeometricMapBuffer::gridSubmapCallback(
  grid_map_msgs::msg::GridMap::ConstSharedPtr grid_submap_msgs)
{
  if (not m_tf_buffer) {
    throw std::runtime_error("Access to tf buffer failure");
  }
  if (not m_grid_map_buffer) {
    throw std::runtime_error("Access to grid map buffer failure");
  }
  auto grid_submap = std::make_unique<grid_map::GridMap>();
  grid_map::GridMapRosConverter::fromMessage(*grid_submap_msgs, *grid_submap);
  std::lock_guard<std::mutex> grid_map_buffer_lock{m_grid_map_mutex};

  if (m_grid_map_buffer->getFrameId().empty()) {
    throw std::runtime_error("Empty frame_id of grid map buffer");
  }
  const bool different_map_frame_id = m_grid_map_buffer->getFrameId() != grid_submap->getFrameId();
  if (different_map_frame_id) {
    geometry_msgs::msg::TransformStamped tfs_msg{};
    try {
      tfs_msg = m_tf_buffer->lookupTransform(
        m_grid_map_buffer->getFrameId(),
        grid_submap->getFrameId(),
        tf2::TimePointZero
      );
    } catch (const tf2::TransformException & e) {
      std::cerr << "[GeometricMapBuffer] " << e.what() << std::endl;
      return;
    }
    grid_map::Position dist_of_map_to_submap;
    dist_of_map_to_submap.x() = tfs_msg.transform.translation.x;
    dist_of_map_to_submap.y() = tfs_msg.transform.translation.y;
    dist_of_map_to_submap += grid_submap->getPosition();
    grid_submap->setPosition(dist_of_map_to_submap);
  }
  const bool result_add_data = m_grid_map_buffer->addDataFrom(*grid_submap, true, true, true);
  if (not result_add_data) {
    throw std::runtime_error("Failed addDataFrom");
  }
  m_grid_map_buffer->setTimestamp(grid_submap->getTimestamp());
}

void GeometricMapBuffer::publishGridMapService(
  std_srvs::srv::Empty::Request::ConstSharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  publishGridMap();
}
}  // namespace geometric_map_buffer
