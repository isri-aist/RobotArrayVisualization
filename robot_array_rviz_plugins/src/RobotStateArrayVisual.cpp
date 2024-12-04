#include "RobotStateArrayVisual.h"

#include "RbdLinkUpdater.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>
#include <rviz_common/visualization_manager.hpp>

using namespace RobotArrayRvizPlugins;

RobotStateArrayVisual::RobotStateArrayVisual() {}

RobotStateArrayVisual::~RobotStateArrayVisual() {}

void RobotStateArrayVisual::reset()
{
  for(auto & robot : robot_list_)
  {
    robot->clear();
  }
}

void RobotStateArrayVisual::reset(int idx)
{
  robot_list_[idx]->clear();
}

void RobotStateArrayVisual::update(const rbd::MultiBody & mb, const std::vector<rbd::MultiBodyConfig> & mbc_list)
{
  // TODO: check if this line is neccessary
  // assert(mb.joint(0).name() == "Root");
  assert(robot_list_.size() == mbc_list.size());

  for(int i = 0; i < robot_list_.size(); i++)
  {
    if(robot_list_[i]->isVisible())
    {
      robot_list_[i]->update(RbdLinkUpdater(&mb, &(mbc_list[i])));
    }
  }
}

void RobotStateArrayVisual::update(int idx, const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc)
{
  if(!robot_list_[idx]->isVisible())
  {
    return;
  }

  assert(mb.joint(0).name() == "Root");

  robot_list_[idx]->update(RbdLinkUpdater(&mb, &mbc));
}

void RobotStateArrayVisual::allocateRobotModel(int num,
                                               Ogre::SceneNode * root_node,
                                               rviz_common::DisplayContext * context,
                                               rviz_common::properties::Property * parent_property)
{
  robot_list_.clear();
  for(auto & robot_property : robot_property_list_)
  {
    delete robot_property;
  }
  robot_property_list_.clear();

  for(int i = 0; i < num; i++)
  {
    std::ostringstream idx_str;
    idx_str << std::setw(3) << std::setfill('0') << i;
    std::string robot_name = "Robot_" + idx_str.str();

    rviz_common::properties::BoolProperty * robot_property = new rviz_common::properties::BoolProperty(
        QString::fromStdString(robot_name), true, "Specifies whether the robot is displayed", parent_property);
    robot_property_list_.push_back(robot_property);
    robot_property->hide();

    robot_list_.push_back(std::make_shared<rviz_default_plugins::robot::Robot>(root_node, context, robot_name, robot_property));
  }
}

void RobotStateArrayVisual::loadRobotModel(const urdf::ModelInterface & urdf_model)
{
  // robot->clear() should be called before robot->load():
  // http://docs.ros.org/jade/api/rviz/html/c++/classrviz_1_1Robot.html#a4b3e851dd812df29f9458afa92e81d3a
  reset();

  for(auto & robot : robot_list_)
  {
    robot->load(urdf_model, true, false);
    robot->setVisualVisible(true);
    robot->setCollisionVisible(false);
    robot->setVisible(true);
  }
}

void RobotStateArrayVisual::loadRobotModel(int idx, const urdf::ModelInterface & urdf_model)
{
  auto & robot = robot_list_[idx];

  // robot->clear() should be called before robot->load():
  // http://docs.ros.org/jade/api/rviz/html/c++/classrviz_1_1Robot.html#a4b3e851dd812df29f9458afa92e81d3a
  robot->clear();

  robot->load(urdf_model, true, false);
  robot->setVisualVisible(true);
  robot->setCollisionVisible(false);
  robot->setVisible(true);
}

void RobotStateArrayVisual::setVisible(int num)
{
  std::cout << "setVisible: " << num << std::endl;
  for(int i = 0; i < robot_list_.size(); i++)
  {
    rviz_common::properties::BoolProperty * robot_property = robot_property_list_[i];
    std::shared_ptr<rviz_default_plugins::robot::Robot> robot = robot_list_[i];
    if(i < num)
    {
      robot_property->show();
      robot->setVisible(robot_property->getBool());
    }
    else
    {
      robot_property->hide();
      robot->setVisible(false);
    }
  }
}

void RobotStateArrayVisual::setColor(int idx, bool original, const Ogre::ColourValue & color)
{
  for(auto & link : robot_list_[idx]->getLinks())
  {
    if(original)
    {
      link.second->unsetColor();
    }
    else
    {
      link.second->setColor(color.r, color.g, color.b);
    }
    link.second->setRobotAlpha(color.a);
  }
}
