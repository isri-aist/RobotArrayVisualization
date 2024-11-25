#pragma once

#include <RBDyn/MultiBodyConfig.h>

#include <rviz_common/display.hpp>
#include <rviz_default_plugins/robot/robot.hpp>

namespace RobotArrayRvizPlugins
{
class RobotStateArrayVisual
{
public:
  RobotStateArrayVisual();

  ~RobotStateArrayVisual();

  void reset();

  void reset(int idx);

  void update(const rbd::MultiBody & mb, const std::vector<rbd::MultiBodyConfig> & mbc_list);

  void update(int idx, const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc);

  void allocateRobotModel(int num,
                          Ogre::SceneNode * root_node,
                          rviz_common::DisplayContext * context,
                          rviz_common::properties::Property * parent_property);

  void loadRobotModel(const urdf::ModelInterface & urdf_model);

  void loadRobotModel(int idx, const urdf::ModelInterface & urdf_model);

  void setVisible(int num);

  void setColor(int idx, bool original, const Ogre::ColourValue & color);

  std::vector<std::shared_ptr<rviz_default_plugins::robot::Robot>> robot_list_;

protected:
  std::vector<rviz_common::properties::BoolProperty *> robot_property_list_;
};
} // namespace RobotArrayRvizPlugins
