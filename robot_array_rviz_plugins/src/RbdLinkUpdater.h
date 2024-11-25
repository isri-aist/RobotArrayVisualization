#pragma once

#include <RBDyn/MultiBodyConfig.h>

#include <rclcpp/rclcpp.hpp>

#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <rviz_default_plugins/robot/link_updater.hpp>

namespace RobotArrayRvizPlugins
{
class RbdLinkUpdater : public rviz_default_plugins::robot::LinkUpdater
{
public:
  RbdLinkUpdater(const rbd::MultiBody * mb, const rbd::MultiBodyConfig * mbc) : mb_(mb), mbc_(mbc) {}

  inline bool getLinkTransforms(const std::string & link_name,
                                Ogre::Vector3 & visual_position,
                                Ogre::Quaternion & visual_orientation,
                                Ogre::Vector3 & collision_position,
                                Ogre::Quaternion & collision_orientation) const override
  {
    // set the link poses
    const auto & bodyIndexByName = mb_->bodyIndexByName();
    if(bodyIndexByName.count(link_name))
    {
      int body_idx = bodyIndexByName.at(link_name);
      // bodyPosW is represented in the world frame,
      // so you don't need to apply the transformation of root link additionally
      const Eigen::Vector3d & pos = mbc_->bodyPosW[body_idx].translation();
      // sva::PTransformd is left handed, so transpose is necessary
      Eigen::Quaterniond quat(mbc_->bodyPosW[body_idx].rotation().transpose());

      visual_position = Ogre::Vector3(pos.x(), pos.y(), pos.z());
      visual_orientation = Ogre::Quaternion(quat.w(), quat.x(), quat.y(), quat.z());

      collision_position = visual_position;
      collision_orientation = visual_orientation;
    }
    else
    {
    //   ROS_ERROR_STREAM("link " << link_name << " not found in the RBDyn robot model.");
      return false;
    }

    return true;
  }

private:
  const rbd::MultiBody * mb_;
  const rbd::MultiBodyConfig * mbc_;
};
} // namespace RobotArrayRvizPlugins
