#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <RBDyn/MultiBodyConfig.h>

#include <robot_array_msgs/msg/robot_state_array.hpp>
#include <robot_array_msgs/msg/robot_description_array.hpp>

#include <OgreColourValue.h>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/display.hpp>
#include <urdf/model.h>

namespace rviz_common
{
namespace properties
{
class StringProperty;
class IntProperty;
class RosTopicProperty;
class EnumProperty;
class FilePickerProperty;
} 
}// namespace rviz

namespace RobotArrayRvizPlugins
{
class RobotStateArrayVisual;
class ColorPropertySet;

class MultiRobotStateArrayDisplay : public rviz_common::Display
{
  Q_OBJECT

protected:
  enum class ColorStyle
  {
    UNIFORM = 0,
    GRADATION_EACH_ROBOT,
    GRADATION_ROBOT_TYPE
  };

public:
  MultiRobotStateArrayDisplay();

  virtual ~MultiRobotStateArrayDisplay();

  virtual void load(const rviz_common::Config & config) override;

  virtual void update(float wall_dt, float ros_dt) override;

  virtual void reset() override;

protected:
  virtual void onInitialize() override;

  virtual void onEnable() override;

  virtual void onDisable() override;

  virtual void fixedFrameChanged() override;

  void robotStateArrayCallback(const robot_array_msgs::msg::RobotStateArray::SharedPtr msg);

  void robotDescriptionCallback(const robot_array_msgs::msg::RobotDescriptionArray::SharedPtr msg);

  void loadUrdfModel();

  std::vector<std::array<Ogre::Real, 4>> getHsbList() const;

  Ogre::ColourValue getInterpColor(double ratio, const std::vector<std::array<Ogre::Real, 4>> & hsb_list) const;

private Q_SLOTS:
  void changedRobotDescriptionTopic();

  void changedRobotDescriptionFile();

  void changedRobotDescriptionSource();

  void changedRobotStateTopic();

  void changedMaxRobotNum();

  void changedColorStyle();

  void changedColor();

protected:
  rviz_common::properties::RosTopicProperty * robot_description_property_;
  rviz_common::properties::EnumProperty * robot_description_source_property_;
  rviz_common::properties::FilePickerProperty * robot_description_file_property_;
  rviz_common::properties::RosTopicProperty * topic_property_;
  rviz_common::properties::IntProperty * robot_num_property_;
  rviz_common::properties::Property * robots_property_;
  rviz_common::properties::EnumProperty * color_style_property_;
  std::vector<std::shared_ptr<ColorPropertySet>> color_property_sets_;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<robot_array_msgs::msg::RobotStateArray>::SharedPtr robot_array_subscriber_;
  rclcpp::Subscription<robot_array_msgs::msg::RobotDescriptionArray>::SharedPtr robot_description_subscriber_;

  std::shared_ptr<RobotStateArrayVisual> visual_;

  std::unordered_map<std::string, urdf::ModelInterfaceSharedPtr> urdf_model_map_;
  std::shared_ptr<robot_array_msgs::msg::RobotDescriptionArray> robot_description_array_;

  std::unordered_map<std::string, rbd::MultiBody> mb_map_;
  std::unordered_map<std::string, rbd::MultiBodyConfig> mbc_map_;

  std::vector<std::string> name_list_;
  std::vector<rbd::MultiBodyConfig> mbc_list_;

  bool robot_inited_ = false;
  bool state_updated_ = false;

  int robot_num_ = 0;
  int msg_num_ = 0;
};
} // namespace RobotArrayRvizPlugins
