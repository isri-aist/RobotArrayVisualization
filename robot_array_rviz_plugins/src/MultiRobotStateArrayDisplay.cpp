#include "MultiRobotStateArrayDisplay.h"

#include <RBDyn/FK.h>
#include <RBDyn/parsers/urdf.h>

#include "ColorPropertySet.h"
#include "RobotStateArrayVisual.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/visualization_manager.h>

using namespace RobotArrayRvizPlugins;

MultiRobotStateArrayDisplay::MultiRobotStateArrayDisplay()
{
  robot_description_property_ = new rviz::StringProperty(
      "Robot Description Map", "robot_description_map",
      "The map from the robot name to the ROS parameter name where the robot URDF model is stored", this,
      SLOT(changedRobotDescription()), this);

  topic_property_ = new rviz::RosTopicProperty(
      "Robot State Array Topic", "robot_state_arr", ros::message_traits::datatype<robot_array_msgs::RobotStateArray>(),
      "The topic on which the robot_array_msgs::RobotStateArray messages are received", this,
      SLOT(changedRobotStateTopic()), this);

  robot_num_property_ =
      new rviz::IntProperty("Max Robot Num", 5, "The maximum number of robots", this, SLOT(changedMaxRobotNum()), this);
  robot_num_property_->setMin(1);
  robot_num_property_->setMax(1000);

  robots_property_ = new rviz::Property("Robots", QVariant(), "", this);

  color_style_property_ = new rviz::EnumProperty("Robot Color Style", "Uniform", "The style of the robot color", this,
                                                 SLOT(changedColorStyle()), this);
  color_style_property_->addOptionStd("Uniform", static_cast<int>(ColorStyle::UNIFORM));
  color_style_property_->addOptionStd("Gradation per each robot", static_cast<int>(ColorStyle::GRADATION_EACH_ROBOT));
  color_style_property_->addOptionStd("Gradation per robot type", static_cast<int>(ColorStyle::GRADATION_ROBOT_TYPE));

  for(int i = 0; i < 2; i++)
  {
    color_property_sets_.push_back(
        std::make_shared<ColorPropertySet>("Color" + std::to_string(i), this, color_style_property_));
  }
}

MultiRobotStateArrayDisplay::~MultiRobotStateArrayDisplay() {}

void MultiRobotStateArrayDisplay::load(const rviz::Config & config)
{
  Display::load(config);
}

void MultiRobotStateArrayDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  if(state_updated_ && robot_inited_)
  {
    state_updated_ = false;
    for(int i = 0; i < robot_num_; i++)
    {
      const std::string & robot_name = name_list_[i];
      visual_->update(i, mb_map_.at(robot_name), mbc_list_[i]);
    }
  }
}

void MultiRobotStateArrayDisplay::reset()
{
  name_list_.clear();
  mbc_list_.clear();
  robot_inited_ = false;
  Display::reset();
  visual_->reset();
  if(isEnabled())
  {
    onEnable();
  }
}

void MultiRobotStateArrayDisplay::onInitialize()
{
  Display::onInitialize();
  visual_ = std::make_shared<RobotStateArrayVisual>();
}

void MultiRobotStateArrayDisplay::onEnable()
{
  Display::onEnable();
  if(!robot_inited_)
  {
    loadUrdfModel();
    robot_inited_ = true;
  }
  changedRobotStateTopic();
  changedColorStyle();
}

void MultiRobotStateArrayDisplay::onDisable()
{
  subscriber_.shutdown();
  Display::onDisable();
}

void MultiRobotStateArrayDisplay::fixedFrameChanged() {}

void MultiRobotStateArrayDisplay::robotStateArrayCallback(const robot_array_msgs::RobotStateArray::ConstPtr & msg)
{
  const std::string & first_frame_id = msg->robot_states[0].root_pose.header.frame_id;

  // assume that the frame_ids of all robot states are the same
  for(int i = 1; i < msg->robot_states.size(); i++)
  {
    assert(first_frame_id == msg->robot_states[i].root_pose.header.frame_id);
  }

  // set name_list_
  std::vector<std::string> prev_name_list = name_list_;
  name_list_.clear();
  std::vector<const robot_array_msgs::RobotState *> robot_state_msg_list;
  for(size_t i = 0; i < std::min(visual_->robot_list_.size(), msg->robot_states.size()); i++)
  {
    const robot_array_msgs::RobotState & robot_state_msg = msg->robot_states[i];
    const std::string & robot_name = robot_state_msg.name;

    if(urdf_model_map_.count(robot_name) == 0)
    {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Not found the URDF for the robot in the message, please check the ROS parameter. (robot_name: "
                   << robot_name << ")");
      continue;
    }

    name_list_.push_back(robot_name);
    robot_state_msg_list.push_back(&robot_state_msg);
  }

  // set robot_num_
  int prev_robot_num = robot_num_;
  robot_num_ = name_list_.size();

  // set mbc_list_ and load robot
  if(name_list_ != prev_name_list)
  {
    mbc_list_.clear();
    for(int i = 0; i < robot_num_; i++)
    {
      const std::string & robot_name = name_list_[i];

      // set mbc_list_
      mbc_list_.push_back(mbc_map_.at(robot_name));

      // load robot model into visual_
      visual_->loadRobotModel(i, *(urdf_model_map_.at(robot_name)));
    }
  }

  // get the transformation from the fixed frame to the message frame
  Ogre::Vector3 frame_pos;
  Ogre::Quaternion frame_quat;
  context_->getFrameManager()->getTransform(first_frame_id, ros::Time(0), frame_pos, frame_quat);
  scene_node_->setPosition(frame_pos);
  scene_node_->setOrientation(frame_quat);

  // update visible settings
  visual_->setVisible(robot_num_);
  if(robot_num_ != prev_robot_num)
  {
    changedColor();
  }

  // update mbc_list_
  for(int i = 0; i < robot_num_; i++)
  {
    const robot_array_msgs::RobotState & robot_state_msg = *(robot_state_msg_list[i]);
    const std::string & robot_name = robot_state_msg.name;

    const auto & mb = mb_map_.at(robot_name);
    auto & mbc = mbc_list_[i];
    const auto & jointIndexByName = mb.jointIndexByName();
    assert(mb.joint(0).name() == "Root");

    if(!visual_->robot_list_[i]->isVisible())
    {
      continue;
    }

    // set the root pose
    std::vector<double> & root_q = mbc.q[0];
    const geometry_msgs::PoseStamped & root_pose_msg = robot_state_msg.root_pose;
    const geometry_msgs::Point & root_pos_msg = root_pose_msg.pose.position;
    const geometry_msgs::Quaternion & root_quat_msg = root_pose_msg.pose.orientation;
    root_q[0] = root_quat_msg.w;
    root_q[1] = root_quat_msg.x;
    root_q[2] = root_quat_msg.y;
    root_q[3] = root_quat_msg.z;
    root_q[4] = root_pos_msg.x;
    root_q[5] = root_pos_msg.y;
    root_q[6] = root_pos_msg.z;

    // set the joint positions
    for(int j = 0; j < robot_state_msg.joint_name_list.size(); j++)
    {
      const std::string & joint_name = robot_state_msg.joint_name_list[j];
      const double & joint_pos = robot_state_msg.joint_pos_list[j];
      if(!jointIndexByName.count(joint_name))
      {
        ROS_ERROR_STREAM("joint " << joint_name << " not found in the RBDyn robot model.");
        continue;
      }

      int joint_idx = jointIndexByName.at(joint_name);
      mbc.q[joint_idx][0] = joint_pos;
      assert(mbc.q[joint_idx].size() == 1);
    }

    // calculate the forward kinematics and upate bodyPosW
    rbd::forwardKinematics(mb, mbc);
  }

  msg_num_++;
  setStatus(rviz::StatusProperty::Ok, "MultiRobotStateArrayDisplay", QString::number(msg_num_) + " messages received");

  state_updated_ = true;
}

void MultiRobotStateArrayDisplay::loadUrdfModel()
{
  // get ROS parameter of map
  std::string map_param_name = robot_description_property_->getStdString();
  XmlRpc::XmlRpcValue urdf_model_map_raw;
  if(!nh_.getParam(map_param_name, urdf_model_map_raw))
  {
    ROS_ERROR_STREAM("Failed to load robot_description_map. (param_name: " << map_param_name << ")");
    return;
  }

  // set map of URDF and RBDyn models
  urdf_model_map_.clear();
  for(const auto & urdf_model_kv : urdf_model_map_raw)
  {
    // get ROS parameter of URDF
    const std::string & robot_name = urdf_model_kv.first;
    const std::string & urdf_param_name = urdf_model_kv.second;
    std::string urdf_content;
    if(!nh_.getParam(urdf_param_name, urdf_content))
    {
      ROS_ERROR_STREAM("Failed to load robot_description. (param_name: " << urdf_param_name << ")");
      return;
    }

    // set URDF model
    std::unique_ptr<urdf::Model> urdf_model(new urdf::Model());
    if(!urdf_model->initString(urdf_content))
    {
      ROS_ERROR_STREAM("Failed to init a urdf model from robot_description. (robot_name: " << robot_name << ")");
      continue;
    }
    urdf_model_map_.emplace(robot_name, std::move(urdf_model));

    // set RBDyn model
    rbd::parsers::ParserResult parse_res = rbd::parsers::from_urdf(urdf_content, false);
    mb_map_.emplace(robot_name, parse_res.mb);
    // parse_res.mbc is initialized by the forward kinematics calculation with the zero positions
    mbc_map_.emplace(robot_name, parse_res.mbc);
  }

  // allocate robot model in visual_
  int max_robot_num = robot_num_property_->getInt();
  visual_->allocateRobotModel(max_robot_num, scene_node_, context_, robots_property_);

  state_updated_ = true;
}

void MultiRobotStateArrayDisplay::changedRobotDescription()
{
  if(isEnabled())
  {
    reset();
  }
}

void MultiRobotStateArrayDisplay::changedRobotStateTopic()
{
  subscriber_.shutdown();

  setStatus(rviz::StatusProperty::Warn, "MultiRobotStateArrayDisplay", "No message received");

  robot_num_ = 0;
  visual_->setVisible(robot_num_);

  subscriber_ =
      nh_.subscribe(topic_property_->getStdString(), 1, &MultiRobotStateArrayDisplay::robotStateArrayCallback, this);

  state_updated_ = true;
}

void MultiRobotStateArrayDisplay::changedMaxRobotNum()
{
  if(isEnabled())
  {
    reset();
  }
}

void MultiRobotStateArrayDisplay::changedColorStyle()
{
  ColorStyle color_style = static_cast<ColorStyle>(color_style_property_->getOptionInt());

  // set the color name and hidden settings
  if(color_style == ColorStyle::UNIFORM)
  {
    color_property_sets_[1]->setHidden(true);
    color_property_sets_[0]->setName("Color");
  }
  else if(color_style == ColorStyle::GRADATION_EACH_ROBOT)
  {
    color_property_sets_[1]->setHidden(false);
    color_property_sets_[0]->setName("First Color");
    color_property_sets_[1]->setName("Last Color");
  }
  else // if(color_style == ColorStyle::GRADATION_ROBOT_TYPE)
  {
    color_property_sets_[1]->setHidden(false);
    color_property_sets_[0]->setName("First Color");
    color_property_sets_[1]->setName("Last Color");
  }

  // the original property is always false when the style is gradation
  if(color_style == ColorStyle::GRADATION_EACH_ROBOT || color_style == ColorStyle::GRADATION_ROBOT_TYPE)
  {
    for(auto & color_property : color_property_sets_)
    {
      color_property->original_property_->setValue(false);
      color_property->original_property_->setHidden(true);
    }
  }
  else
  {
    for(auto & color_property : color_property_sets_)
    {
      color_property->original_property_->setHidden(false);
    }
  }

  color_style_property_->expand();
  color_property_sets_[0]->label_property_->expand();
  color_property_sets_[1]->label_property_->expand();

  changedColor();
}

std::vector<std::array<Ogre::Real, 4>> MultiRobotStateArrayDisplay::getHsbList() const
{
  // convert RGB to HSV
  int color_num = color_property_sets_.size();
  std::vector<std::array<Ogre::Real, 4>> hsb_list(color_num);
  for(int i = 0; i < color_num; i++)
  {
    const Ogre::ColourValue & color = color_property_sets_[i]->getColor();
    color.getHSB(&(hsb_list[i][0]), &(hsb_list[i][1]), &(hsb_list[i][2]));
    hsb_list[i][3] = color.a;
  }

  return hsb_list;
}

Ogre::ColourValue MultiRobotStateArrayDisplay::getInterpColor(
    double ratio,
    const std::vector<std::array<Ogre::Real, 4>> & hsb_list) const
{
  // calculate the interpolated idx and ratio
  int color_num = hsb_list.size();
  double interp_value = (color_num - 1) * ratio;
  int interp_idx = std::floor(interp_value);
  double interp_ratio = interp_value - interp_idx;
  // special treatment for the last
  if(ratio == 1.0)
  {
    interp_idx = color_num - 2;
    interp_ratio = 1.0;
  }

  // calculate the interpolated color
  std::array<Ogre::Real, 4> interp_hsb;
  Ogre::ColourValue interp_color;
  for(int j = 0; j < interp_hsb.size(); j++)
  {
    interp_hsb[j] = (1 - interp_ratio) * hsb_list[interp_idx][j] + interp_ratio * hsb_list[interp_idx + 1][j];
  }
  interp_color.setHSB(interp_hsb[0], interp_hsb[1], interp_hsb[2]);
  interp_color.a = interp_hsb[3];

  return interp_color;
}

void MultiRobotStateArrayDisplay::changedColor()
{
  if(robot_num_ == 0)
  {
    return;
  }

  ColorStyle color_style = static_cast<ColorStyle>(color_style_property_->getOptionInt());

  if(color_style == ColorStyle::UNIFORM)
  {
    for(int i = 0; i < robot_num_; i++)
    {
      // all robots
      visual_->setColor(i, color_property_sets_[0]->getOriginal(), color_property_sets_[0]->getColor());
    }
  }
  else if(color_style == ColorStyle::GRADATION_EACH_ROBOT)
  {
    if(robot_num_ == 1)
    {
      visual_->setColor(0, false, color_property_sets_[0]->getColor());
    }
    else
    {
      for(int i = 0; i < robot_num_; i++)
      {
        double ratio = static_cast<double>(i) / (robot_num_ - 1);
        visual_->setColor(i, false, getInterpColor(ratio, getHsbList()));
      }
    }
  }
  else // if(color_style == ColorStyle::GRADATION_ROBOT_TYPE)
  {
    // set uniq_name_list
    std::vector<std::string> uniq_name_list;
    for(const auto & name : name_list_)
    {
      if(std::find(uniq_name_list.begin(), uniq_name_list.end(), name) == uniq_name_list.end())
      {
        uniq_name_list.push_back(name);
      }
    }

    int name_num = uniq_name_list.size();
    for(int i = 0; i < robot_num_; i++)
    {
      double ratio = 0.0;
      if(name_num > 1)
      {
        const auto & iter = std::find(uniq_name_list.begin(), uniq_name_list.end(), name_list_[i]);
        int name_idx = std::distance(uniq_name_list.begin(), iter);
        ratio = static_cast<double>(name_idx) / (name_num - 1);
      }
      visual_->setColor(i, false, getInterpColor(ratio, getHsbList()));
    }
  }

  state_updated_ = true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RobotArrayRvizPlugins::MultiRobotStateArrayDisplay, rviz::Display)
