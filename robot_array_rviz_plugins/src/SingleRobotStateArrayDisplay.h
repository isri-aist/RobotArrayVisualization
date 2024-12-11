#pragma once

#include <RBDyn/MultiBodyConfig.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <robot_array_msgs/msg/robot_state_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/display.hpp>
#include <urdf/model.h>

namespace rviz_common
{
namespace properties
{
class StringProperty;
class BoolProperty;
class IntProperty;
class RosTopicProperty;
class EnumProperty;
}
} // namespace rviz

namespace RobotArrayRvizPlugins
{
class RobotStateArrayVisual;
class ColorPropertySet;

class SingleRobotStateArrayDisplay : public rviz_common::Display
{
  Q_OBJECT

protected:
  enum class ColorStyle
  {
    UNIFORM = 0,
    FIRST,
    FIRST_LAST,
    GRADATION
  };

public:
  SingleRobotStateArrayDisplay();

  virtual ~SingleRobotStateArrayDisplay();

  virtual void load(const rviz_common::Config & config) override;

  virtual void update(float wall_dt, float ros_dt) override;

  virtual void reset() override;

protected:
  virtual void onInitialize() override;

  virtual void onEnable() override;

  virtual void onDisable() override;

  virtual void fixedFrameChanged() override;

  void robotStateArrayCallback(const robot_array_msgs::msg::RobotStateArray::SharedPtr msg);

  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);

  void loadUrdfModel();

private Q_SLOTS:
  void changedRobotDescriptionTopic();

  void changedRobotStateTopic();

  void changedMaxRobotNum();

  void changedColorStyle();

  void changedColor();

protected:
  rviz_common::properties::RosTopicProperty * robot_description_property_;
  rviz_common::properties::RosTopicProperty * topic_property_;
  rviz_common::properties::BoolProperty * check_name_property_;
  rviz_common::properties::IntProperty * robot_num_property_;
  rviz_common::properties::Property * robots_property_;
  rviz_common::properties::EnumProperty * color_style_property_;
  std::vector<std::shared_ptr<ColorPropertySet>> color_property_sets_;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<robot_array_msgs::msg::RobotStateArray>::SharedPtr robot_array_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;

  std::shared_ptr<RobotStateArrayVisual> visual_;

  urdf::ModelInterfaceSharedPtr urdf_model_;
  std::string urdf_content_;

  rbd::MultiBody mb_;
  std::vector<rbd::MultiBodyConfig> mbc_list_;

  std::string name_;

  bool robot_inited_ = false;
  bool state_updated_ = false;

  int robot_num_ = 0;
  int msg_num_ = 0;
};
} // namespace robot_array_rviz_plugins