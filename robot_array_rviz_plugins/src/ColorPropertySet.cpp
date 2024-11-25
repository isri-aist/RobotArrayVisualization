#include "ColorPropertySet.h"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

using namespace RobotArrayRvizPlugins;

ColorPropertySet::ColorPropertySet(const std::string & name, rviz_common::Display * display, rviz_common::properties::Property * parent_property)
{
  label_property_ = new rviz_common::properties::Property(QString::fromStdString(name), QVariant(), "", parent_property);
  label_property_->expand();

  original_property_ = new rviz_common::properties::BoolProperty("Original Color", true, "Whether the original color is used",
                                              label_property_, SLOT(changedOriginal()), this);

  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(150, 50, 150), "The color of the robot links",
                                            label_property_, SLOT(changedColor()), display);
  color_property_->hide();

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 1.0f, "The alpha of the robot links", label_property_,
                                            SLOT(changedColor()), display);
}

void ColorPropertySet::setName(const std::string & name)
{
  label_property_->setName(QString::fromStdString(name));
}

void ColorPropertySet::setHidden(bool hidden)
{
  label_property_->setHidden(hidden);
}

bool ColorPropertySet::getOriginal()
{
  return original_property_->getBool();
}

Ogre::ColourValue ColorPropertySet::getColor()
{
  Ogre::ColourValue color;
  color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();
  return color;
}

void ColorPropertySet::changedOriginal()
{
  color_property_->setHidden(original_property_->getBool());
  color_property_->changed();
}
