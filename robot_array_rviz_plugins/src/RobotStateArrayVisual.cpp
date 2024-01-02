#include "RobotStateArrayVisual.h"

#include "RbdLinkUpdater.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/visualization_manager.h>

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
  assert(mb.joint(0).name() == "Root");
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
                                               rviz::DisplayContext * context,
                                               rviz::Property * parent_property)
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

    rviz::BoolProperty * robot_property = new rviz::BoolProperty(
        QString::fromStdString(robot_name), true, "Specifies whether the robot is displayed", parent_property);
    robot_property_list_.push_back(robot_property);
    robot_property->hide();

    robot_list_.push_back(std::make_shared<rviz::Robot>(root_node, context, robot_name, robot_property));
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
    robot->setVisible(false);
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
  robot->setVisible(false);
}

void RobotStateArrayVisual::setVisible(int num)
{
  for(int i = 0; i < robot_list_.size(); i++)
  {
    rviz::BoolProperty * robot_property = robot_property_list_[i];
    std::shared_ptr<rviz::Robot> robot = robot_list_[i];
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
