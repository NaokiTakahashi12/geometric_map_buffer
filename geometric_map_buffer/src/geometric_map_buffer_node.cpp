#include <memory>
#include <utility>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/header.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <grid_map_ros/grid_map_ros.hpp>

#include <geometric_map_buffer/geometric_map_buffer.hpp>
#include <geometric_map_buffer_node_parameters.hpp>


namespace geometric_map_buffer
{
class GeometricMapBufferNode : public rclcpp::Node
{
public:
  GeometricMapBufferNode(const rclcpp::NodeOptions &);
  ~GeometricMapBufferNode();

private:
  static constexpr char m_this_node_name[] = "geometric_map_buffer_node";

  std::unique_ptr<geometric_map_buffer_node::Params> m_params;
  std::unique_ptr<geometric_map_buffer_node::ParamListener> m_param_listener;
  GeometricMapBuffer::UniquePtr m_geometric_map_buffer;
};

GeometricMapBufferNode::GeometricMapBufferNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node(m_this_node_name, node_options)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << m_this_node_name);

  m_param_listener = std::make_unique<geometric_map_buffer_node::ParamListener>(
    this->get_node_parameters_interface()
  );
  m_params = std::make_unique<geometric_map_buffer_node::Params>(
    m_param_listener->get_params()
  );

  m_geometric_map_buffer = std::make_unique<GeometricMapBuffer>(*this, "~");

  auto grid_map = std::make_unique<grid_map::GridMap>(grid_map::GridMap({"elevation"}));
  grid_map->setFrameId(m_params->frame_id);
  grid_map->setGeometry(
    grid_map::Length(
      m_params->geometry.length.x,
      m_params->geometry.length.y
    ),
    m_params->geometry.resolution);
  grid_map->setPosition(
    grid_map::Position(
      m_params->position.x,
      m_params->position.y
    )
  );
  m_geometric_map_buffer->publishInitialGridMap(std::move(grid_map));
}

GeometricMapBufferNode::~GeometricMapBufferNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << m_this_node_name);
}
}  // namespace geometric_map_buffer

RCLCPP_COMPONENTS_REGISTER_NODE(geometric_map_buffer::GeometricMapBufferNode)
