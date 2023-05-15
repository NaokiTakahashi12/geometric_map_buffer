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

#include <memory>
#include <string>
#include <array>
#include <algorithm>
#include <utility>
#include <chrono>
#include <filesystem>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometric_map_buffer/geometric_map_buffer.hpp>
#include <geometric_map_buffer/grid_map_converter.hpp>

#include <geometric_map_server_node_parameters.hpp>


namespace geometric_map_server
{
namespace buffer = geometric_map_buffer;
using GridMapUniquePtr = buffer::GeometricMapBuffer::GridMapUniquePtr;

struct BuildGridMapParameters
{
  std::string image_file_path;
  double resolution;
  grid_map::Position position;
  geometric_map_buffer::grid_map_converter::BuildGridMapLayerParameters layer_params;
};

class GeometricMapServerNode : public rclcpp::Node
{
public:
  explicit GeometricMapServerNode(const rclcpp::NodeOptions &);
  ~GeometricMapServerNode();

private:
  static constexpr char m_this_node_name[] = "geometric_map_server_node";

  buffer::GeometricMapBuffer::UniquePtr m_geometric_map_buffer;

  std::unique_ptr<geometric_map_server_node::ParamListener> m_param_listener;
  std::unique_ptr<geometric_map_server_node::Params> m_params;

  rclcpp::TimerBase::SharedPtr m_track_frame_timer;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr m_track_grid_submap_publisher;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;

  BuildGridMapParameters loadGridMapImageParams(const std::string & file_path);
  GridMapUniquePtr loadGridMapFromMapInfo(const std::string & file_path);

  void trackFrameCallback();
};

GeometricMapServerNode::GeometricMapServerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node(m_this_node_name, node_options),
  m_geometric_map_buffer(nullptr),
  m_param_listener(nullptr),
  m_params(nullptr),
  m_track_frame_timer(nullptr),
  m_track_grid_submap_publisher(nullptr),
  m_tf_buffer(nullptr),
  m_tf_listener(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << m_this_node_name);

  m_param_listener = std::make_unique<geometric_map_server_node::ParamListener>(
    this->get_node_parameters_interface()
  );
  m_params = std::make_unique<geometric_map_server_node::Params>(
    m_param_listener->get_params()
  );

  auto grid_map_from_image = loadGridMapFromMapInfo(m_params->map_info);
  grid_map_from_image->setFrameId(m_params->map_frame_id);
  m_geometric_map_buffer = std::make_unique<buffer::GeometricMapBuffer>(*this);
  m_geometric_map_buffer->publishInitialGridMap(std::move(grid_map_from_image));
  m_geometric_map_buffer->enablePublishGridMapService(*this);

  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);

  if (not m_params->track_grid_submap.frame_id.empty()) {
    m_track_grid_submap_publisher = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "~/track_grid_submap",
      rclcpp::QoS(2)
    );
    const unsigned int track_frame_timer_milliseconds =
      1e3 / m_params->track_grid_submap.publish_frequency;
    m_track_frame_timer = this->create_wall_timer(
      std::chrono::milliseconds(
        track_frame_timer_milliseconds
      ),
      std::bind(
        &GeometricMapServerNode::trackFrameCallback,
        this
      )
    );
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Enabled track grid submap ("
        << m_params->track_grid_submap.frame_id
        << ")"
    );
  }
}

GeometricMapServerNode::~GeometricMapServerNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << m_this_node_name);
}

std::string getBaseDir(const std::string & path_string)
{
  std::filesystem::path path{path_string};
  return path.parent_path();
}

BuildGridMapParameters GeometricMapServerNode::loadGridMapImageParams(
  const std::string & file_path
)
{
  if (file_path.empty()) {
    throw std::runtime_error("Load construct grid map parameter file path is empty");
  }
  BuildGridMapParameters build_params;
  std::array<float, 3> map_origin{0.f, 0.f, 0.f};
  YAML::Node node{YAML::LoadFile(file_path)};

  if (node["origin"]) {
    map_origin = node["origin"].as<std::array<float, 3>>();
  }
  build_params.position(0) = map_origin[0];
  build_params.position(1) = map_origin[1];
  build_params.layer_params.lower_value = map_origin[2] + node["lower"].as<float>();
  build_params.layer_params.upper_value = map_origin[2] + node["upper"].as<float>();
  build_params.layer_params.alpha_threshold = node["alpha_threshold"].as<double>();
  build_params.image_file_path =
    getBaseDir(file_path) + "/" + node["image"].as<std::string>();
  build_params.resolution = node["resolution"].as<double>();
  build_params.layer_params.layer_name = node["layer"].as<std::string>();
  return build_params;
}

GridMapUniquePtr GeometricMapServerNode::loadGridMapFromMapInfo(const std::string & file_path)
{
  if (file_path.empty()) {
    throw std::runtime_error("Load image file path is empty");
  }
  const auto construct_grid_map_param = loadGridMapImageParams(file_path);
  const cv::Mat cv_mat = cv::imread(construct_grid_map_param.image_file_path);

  if (cv_mat.empty()) {
    throw std::runtime_error("Failed to read " + construct_grid_map_param.image_file_path);
  }
  GridMapUniquePtr grid_map = std::make_unique<grid_map::GridMap>(
    grid_map::GridMap({"elevation"})
  );
  grid_map::GridMapCvConverter::initializeFromImage(
    cv_mat,
    construct_grid_map_param.resolution,
    *grid_map,
    construct_grid_map_param.position
  );
  buffer::grid_map_converter::addLayerFromImage(
    *grid_map,
    cv_mat,
    construct_grid_map_param.layer_params
  );
  return grid_map;
}

void GeometricMapServerNode::trackFrameCallback()
{
  if (not m_track_grid_submap_publisher) {
    throw std::runtime_error("m_track_grid_submap_publisher is null");
  }
  if (not m_tf_buffer) {
    throw std::runtime_error("m_tf_buffer is null");
  }
  if (not m_params) {
    throw std::runtime_error("m_params is null");
  }
  geometry_msgs::msg::TransformStamped tfs_msg{};
  try {
    tfs_msg = m_tf_buffer->lookupTransform(
      m_params->map_frame_id,
      m_params->track_grid_submap.frame_id,
      tf2::TimePointZero
    );
  } catch (const tf2::TransformException & te) {
    RCLCPP_WARN_STREAM(this->get_logger(), te.what());
    return;
  }
  const grid_map::Position position = grid_map::Position(
    {
      tfs_msg.transform.translation.x,
      tfs_msg.transform.translation.y
    }
  );
  const grid_map::Length length = grid_map::Length(
    {
      m_params->track_grid_submap.length.x,
      m_params->track_grid_submap.length.y
    }
  );
  grid_map_msgs::msg::GridMap::UniquePtr grid_submap_msg;
  grid_submap_msg = grid_map::GridMapRosConverter::toMessage(
    *m_geometric_map_buffer->getGridSubmap(
      position,
      length
    )
  );
  m_track_grid_submap_publisher->publish(std::move(grid_submap_msg));
}
}  // namespace geometric_map_server

RCLCPP_COMPONENTS_REGISTER_NODE(geometric_map_server::GeometricMapServerNode)
