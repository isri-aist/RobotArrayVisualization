#pragma once

#include <RBDyn/FK.h>
#include <RBDyn/MultiBodyConfig.h>

#include <robot_array_msgs/RobotStateArray.h>

#include "SingleRobotStateArrayVisual.h"

#include <rviz/display.h>
#include <urdf/model.h>

namespace rviz
{
class StringProperty;
class BoolProperty;
class IntProperty;
class FloatProperty;
class RosTopicProperty;
class EnumProperty;
class ColorProperty;
} // namespace rviz

namespace RobotArrayRvizPlugins
{
class SingleRobotStateArrayDisplay;

class ColorPropertySet : public rviz::Property
{
  Q_OBJECT

public:
  ColorPropertySet(const std::string & name, SingleRobotStateArrayDisplay * display, rviz::Property * parent_property);

  void setName(const std::string & name);

  void setHidden(bool hidden);

  bool getOriginal();

  Ogre::ColourValue getColor();

private Q_SLOTS:
  void changedOriginal();

protected:
  SingleRobotStateArrayDisplay * display_;

public:
  rviz::Property * label_property_;
  rviz::BoolProperty * original_property_;
  rviz::ColorProperty * color_property_;
  rviz::FloatProperty * alpha_property_;
};

class SingleRobotStateArrayDisplay : public rviz::Display
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

private Q_SLOTS:
  void changedRobotDescription();

  void changedRobotStateTopic();

  void changedMaxRobotNum();

  void changedColorStyle();

  void changedColor();

protected:
  rviz::StringProperty * robot_description_property_;
  rviz::RosTopicProperty * topic_property_;
  rviz::BoolProperty * check_name_property_;
  rviz::IntProperty * robot_num_property_;
  rviz::Property * robots_property_;
  rviz::EnumProperty * color_style_property_;
  std::vector<std::shared_ptr<ColorPropertySet>> color_property_sets_;

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::shared_ptr<SingleRobotStateArrayVisual> visual_;

  urdf::ModelInterfaceSharedPtr urdf_model_;

  rbd::MultiBody mb_;
  std::vector<rbd::MultiBodyConfig> mbc_list_;

  std::string name_;

  bool robot_inited_ = false;
  bool state_updated_ = false;

  int robot_num_ = 0;
  int msg_num_ = 0;
};
} // namespace RobotArrayRvizPlugins
