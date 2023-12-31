#pragma once

#include <OgreColourValue.h>
#include <rviz/properties/property.h>

namespace rviz
{
class BoolProperty;
class FloatProperty;
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
} // namespace RobotArrayRvizPlugins
