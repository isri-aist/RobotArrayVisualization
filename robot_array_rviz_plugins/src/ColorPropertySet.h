#pragma once

#include <OgreColourValue.h>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>

namespace rviz_common
{
namespace properties
{
class BoolProperty;
class FloatProperty;
class ColorProperty;
}
} // namespace rviz_common

namespace RobotArrayRvizPlugins
{
class ColorPropertySet : public rviz_common::properties::Property
{
  Q_OBJECT

public:
  ColorPropertySet(const std::string & name, rviz_common::Display * display, rviz_common::properties::Property * parent_property);

  void setName(const std::string & name);

  void setHidden(bool hidden);

  bool getOriginal();

  Ogre::ColourValue getColor();

private Q_SLOTS:
  void changedOriginal();

public:
  rviz_common::properties::Property * label_property_;
  rviz_common::properties::BoolProperty * original_property_;
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
};
} // namespace RobotArrayRvizPlugins
