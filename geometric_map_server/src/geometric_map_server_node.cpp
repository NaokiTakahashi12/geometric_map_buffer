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
#include <sensor_msgs/msg/image.hpp>
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

  BuildGridMapParameters loadGridMapImageParams(const std::string & file_path);
  GridMapUniquePtr loadGridMapFromMapInfo(const std::string & file_path);
};

GeometricMapServerNode::GeometricMapServerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node(m_this_node_name, node_options),
  m_geometric_map_buffer(nullptr),
  m_param_listener(nullptr),
  m_params(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << m_this_node_name);

  m_param_listener = std::make_unique<geometric_map_server_node::ParamListener>(
    this->get_node_parameters_interface()
  );
  m_params = std::make_unique<geometric_map_server_node::Params>(
    m_param_listener->get_params()
  );

  auto grid_map_from_image = loadGridMapFromMapInfo(m_params->map_info_path);
  grid_map_from_image->setFrameId(m_params->map_frame_id);
  m_geometric_map_buffer = std::make_unique<buffer::GeometricMapBuffer>(*this);
  m_geometric_map_buffer->publishInitialGridMap(std::move(grid_map_from_image));
  m_geometric_map_buffer->enablePublishGridMapService(*this);
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
  BuildGridMapParameters param;
  std::array<float, 3> map_origin{0.f, 0.f, 0.f};
  YAML::Node node{YAML::LoadFile(file_path)};

  if (node["origin"]) {
    map_origin = node["origin"].as<std::array<float, 3>>();
  }
  param.position(0) = map_origin[0];
  param.position(1) = map_origin[1];
  param.layer_params.lower_value = map_origin[2] + node["lower"].as<float>();
  param.layer_params.upper_value = map_origin[2] + node["upper"].as<float>();
  param.layer_params.alpha_threshold = node["alpha_threshold"].as<double>();
  param.image_file_path =
    getBaseDir(file_path) + "/" + node["image"].as<std::string>();
  param.resolution = node["resolution"].as<double>();
  param.layer_params.layer_name = node["layer"].as<std::string>();
  return param;
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
}  // namespace geometric_map_server

RCLCPP_COMPONENTS_REGISTER_NODE(geometric_map_server::GeometricMapServerNode)
