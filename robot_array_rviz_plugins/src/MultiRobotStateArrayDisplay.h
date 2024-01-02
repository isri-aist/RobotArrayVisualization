#pragma once

#include <RBDyn/MultiBodyConfig.h>

#include <robot_array_msgs/RobotStateArray.h>

#include <OgreColourValue.h>
#include <rviz/display.h>
#include <urdf/model.h>

namespace rviz
{
class StringProperty;
class IntProperty;
class RosTopicProperty;
class EnumProperty;
} // namespace rviz

namespace RobotArrayRvizPlugins
{
class SingleRobotStateArrayVisual;
class ColorPropertySet;

class MultiRobotStateArrayDisplay : public rviz::Display
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

  virtual void load(const rviz::Config & config) override;

  virtual void update(float wall_dt, float ros_dt) override;

  virtual void reset() override;

protected:
  virtual void onInitialize() override;

  virtual void onEnable() override;

  virtual void onDisable() override;

  virtual void fixedFrameChanged() override;

  void robotStateArrayCallback(const robot_array_msgs::RobotStateArray::ConstPtr & msg);

  void loadUrdfModel();

  std::vector<std::array<Ogre::Real, 4>> getHsbList() const;

  Ogre::ColourValue getInterpColor(double ratio, const std::vector<std::array<Ogre::Real, 4>> & hsb_list) const;

private Q_SLOTS:
  void changedRobotDescription();

  void changedRobotStateTopic();

  void changedMaxRobotNum();

  void changedColorStyle();

  void changedColor();

protected:
  rviz::StringProperty * robot_description_property_;
  rviz::RosTopicProperty * topic_property_;
  rviz::IntProperty * robot_num_property_;
  rviz::Property * robots_property_;
  rviz::EnumProperty * color_style_property_;
  std::vector<std::shared_ptr<ColorPropertySet>> color_property_sets_;

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::shared_ptr<SingleRobotStateArrayVisual> visual_;

  std::unordered_map<std::string, urdf::ModelInterfaceSharedPtr> urdf_model_map_;

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
