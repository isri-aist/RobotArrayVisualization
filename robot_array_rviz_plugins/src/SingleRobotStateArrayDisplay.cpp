#include "SingleRobotStateArrayDisplay.h"

#include <RBDyn/FK.h>
#include <RBDyn/parsers/urdf.h>

#include "ColorPropertySet.h"
#include "SingleRobotStateArrayVisual.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/visualization_manager.h>

using namespace RobotArrayRvizPlugins;

SingleRobotStateArrayDisplay::SingleRobotStateArrayDisplay()
{
  robot_description_property_ = new rviz::StringProperty(
      "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
      this, SLOT(changedRobotDescription()), this);

  topic_property_ = new rviz::RosTopicProperty(
      "Robot State Array Topic", "robot_state_arr", ros::message_traits::datatype<robot_array_msgs::RobotStateArray>(),
      "The topic on which the robot_array_msgs::RobotStateArray messages are received", this,
      SLOT(changedRobotStateTopic()), this);

  check_name_property_ =
      new rviz::BoolProperty("Check Name Consistency", false,
                             "Whether to devisualize robots with inconsistent names between URDF and message", this);

  robot_num_property_ =
      new rviz::IntProperty("Max Robot Num", 5, "The maximum number of robots", this, SLOT(changedMaxRobotNum()), this);
  robot_num_property_->setMin(1);
  robot_num_property_->setMax(1000);

  robots_property_ = new rviz::Property("Robots", QVariant(), "", this);

  color_style_property_ = new rviz::EnumProperty("Robot Color Style", "Uniform", "The style of the robot color", this,
                                                 SLOT(changedColorStyle()), this);
  color_style_property_->addOptionStd("Uniform", static_cast<int>(ColorStyle::UNIFORM));
  color_style_property_->addOptionStd("First", static_cast<int>(ColorStyle::FIRST));
  color_style_property_->addOptionStd("First and Last", static_cast<int>(ColorStyle::FIRST_LAST));
  color_style_property_->addOptionStd("Gradation", static_cast<int>(ColorStyle::GRADATION));

  for(int i = 0; i < 2; i++)
  {
    color_property_sets_.push_back(
        std::make_shared<ColorPropertySet>("Color" + std::to_string(i), this, color_style_property_));
  }
}

SingleRobotStateArrayDisplay::~SingleRobotStateArrayDisplay() {}

void SingleRobotStateArrayDisplay::load(const rviz::Config & config)
{
  Display::load(config);
}

void SingleRobotStateArrayDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  if(state_updated_ && robot_inited_)
  {
    state_updated_ = false;
    visual_->update(mb_, mbc_list_);
  }
}

void SingleRobotStateArrayDisplay::reset()
{
  robot_inited_ = false;
  Display::reset();
  visual_->reset();
  if(isEnabled())
  {
    onEnable();
  }
}

void SingleRobotStateArrayDisplay::onInitialize()
{
  Display::onInitialize();
  visual_ = std::make_shared<SingleRobotStateArrayVisual>();
}

void SingleRobotStateArrayDisplay::onEnable()
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

void SingleRobotStateArrayDisplay::onDisable()
{
  subscriber_.shutdown();
  Display::onDisable();
}

void SingleRobotStateArrayDisplay::fixedFrameChanged() {}

void SingleRobotStateArrayDisplay::robotStateArrayCallback(const robot_array_msgs::RobotStateArray::ConstPtr & msg)
{
  const std::string & first_frame_id = msg->robot_states[0].root_pose.header.frame_id;

  // assume that the frame_ids of all robot states are the same
  for(int i = 1; i < msg->robot_states.size(); i++)
  {
    assert(first_frame_id == msg->robot_states[i].root_pose.header.frame_id);
  }

  // get the transformation from the fixed frame to the message frame
  Ogre::Vector3 frame_pos;
  Ogre::Quaternion frame_quat;
  context_->getFrameManager()->getTransform(first_frame_id, ros::Time(0), frame_pos, frame_quat);
  scene_node_->setPosition(frame_pos);
  scene_node_->setOrientation(frame_quat);

  const auto & jointIndexByName = mb_.jointIndexByName();
  assert(mb_.joint(0).name() == "Root");

  // update visible settings
  int prev_robot_num = robot_num_;
  robot_num_ = std::min(mbc_list_.size(), msg->robot_states.size());
  visual_->setVisible(robot_num_);
  if(robot_num_ != prev_robot_num)
  {
    changedColor();
  }

  // update mbc_list_
  for(int i = 0; i < robot_num_; i++)
  {
    const robot_array_msgs::RobotState & robot_state_msg = msg->robot_states[i];

    if(check_name_property_->getBool() && robot_state_msg.name.find(name_) == std::string::npos)
    {
      visual_->robot_list_[i]->setVisible(false);
    }

    if(!visual_->robot_list_[i]->isVisible())
    {
      continue;
    }

    // set the root pose
    std::vector<double> & root_q = mbc_list_[i].q[0];
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
      mbc_list_[i].q[joint_idx][0] = joint_pos;
      assert(mbc_list_[i].q[joint_idx].size() == 1);
    }

    // calculate the forward kinematics and upate bodyPosW
    rbd::forwardKinematics(mb_, mbc_list_[i]);
  }

  msg_num_++;
  setStatus(rviz::StatusProperty::Ok, "SingleRobotStateArrayDisplay", QString::number(msg_num_) + " messages received");

  state_updated_ = true;
}

void SingleRobotStateArrayDisplay::loadUrdfModel()
{
  // get urdf_content
  std::string param_name = robot_description_property_->getStdString();
  std::string urdf_content;
  if(!nh_.getParam(param_name, urdf_content))
  {
    ROS_ERROR_STREAM("Failed to load robot_description. (param_name: " << param_name << ")");
    return;
  }

  // set urdf_model
  std::unique_ptr<urdf::Model> urdf_model(new urdf::Model());
  if(!urdf_model->initString(urdf_content))
  {
    ROS_ERROR("Failed to init a urdf model from robot_description.");
    return;
  }

  int max_robot_num = robot_num_property_->getInt();

  // allocate robot model in visual_
  visual_->allocateRobotModel(max_robot_num, scene_node_, context_, robots_property_);

  // load robot model into visual_
  urdf_model_ = std::move(urdf_model);
  visual_->loadRobotModel(*urdf_model_);

  // load robot model into mb_ and mbc_list_
  rbd::parsers::ParserResult parse_res = rbd::parsers::from_urdf(urdf_content, false);
  mb_ = parse_res.mb;
  name_ = parse_res.name;
  // parse_res.mbc is initialized by the forward kinematics calculation with the zero positions
  mbc_list_.resize(max_robot_num, parse_res.mbc);

  state_updated_ = true;
}

void SingleRobotStateArrayDisplay::changedRobotDescription()
{
  if(isEnabled())
  {
    reset();
  }
}

void SingleRobotStateArrayDisplay::changedRobotStateTopic()
{
  subscriber_.shutdown();

  setStatus(rviz::StatusProperty::Warn, "SingleRobotStateArrayDisplay", "No message received");

  robot_num_ = 0;
  visual_->setVisible(robot_num_);

  subscriber_ =
      nh_.subscribe(topic_property_->getStdString(), 1, &SingleRobotStateArrayDisplay::robotStateArrayCallback, this);

  state_updated_ = true;
}

void SingleRobotStateArrayDisplay::changedMaxRobotNum()
{
  if(isEnabled())
  {
    reset();
  }
}

void SingleRobotStateArrayDisplay::changedColorStyle()
{
  ColorStyle color_style = static_cast<ColorStyle>(color_style_property_->getOptionInt());

  // set the color name and hidden settings
  if(color_style == ColorStyle::UNIFORM)
  {
    color_property_sets_[1]->setHidden(true);
    color_property_sets_[0]->setName("Color");
  }
  else if(color_style == ColorStyle::FIRST)
  {
    color_property_sets_[1]->setHidden(false);
    color_property_sets_[0]->setName("First Color");
    color_property_sets_[1]->setName("Other Color");
  }
  else if(color_style == ColorStyle::FIRST_LAST)
  {
    color_property_sets_[1]->setHidden(false);
    color_property_sets_[0]->setName("First & Last Color");
    color_property_sets_[1]->setName("Other Color");
  }
  else if(color_style == ColorStyle::GRADATION)
  {
    color_property_sets_[1]->setHidden(false);
    color_property_sets_[0]->setName("First Color");
    color_property_sets_[1]->setName("Last Color");
  }

  // the original property is always false when the style is gradation
  if(color_style == ColorStyle::GRADATION)
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

void SingleRobotStateArrayDisplay::changedColor()
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
  else if(color_style == ColorStyle::FIRST)
  {
    for(int i = 0; i < robot_num_; i++)
    {
      // first robot
      if(i == 0)
      {
        visual_->setColor(i, color_property_sets_[0]->getOriginal(), color_property_sets_[0]->getColor());
      }
      else
      {
        // other robots
        visual_->setColor(i, color_property_sets_[1]->getOriginal(), color_property_sets_[1]->getColor());
      }
    }
  }
  else if(color_style == ColorStyle::FIRST_LAST)
  {
    for(int i = 0; i < robot_num_; i++)
    {
      if(i == 0 || i == robot_num_ - 1)
      {
        // first and last robots
        visual_->setColor(i, color_property_sets_[0]->getOriginal(), color_property_sets_[0]->getColor());
      }
      else
      {
        // other robots
        visual_->setColor(i, color_property_sets_[1]->getOriginal(), color_property_sets_[1]->getColor());
      }
    }
  }
  else if(color_style == ColorStyle::GRADATION)
  {
    if(robot_num_ == 1)
    {
      visual_->setColor(0, false, color_property_sets_[0]->getColor());
    }
    else
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

      for(int i = 0; i < robot_num_; i++)
      {
        // calculate the interpolated idx and ratio
        double interp_value = (color_num - 1) * (static_cast<double>(i) / (robot_num_ - 1));
        int interp_idx = std::floor(interp_value);
        double interp_ratio = interp_value - interp_idx;
        // special treatment for the last
        if(i == robot_num_ - 1)
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

        // set the color
        visual_->setColor(i, false, interp_color);
      }
    }
  }

  state_updated_ = true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RobotArrayRvizPlugins::SingleRobotStateArrayDisplay, rviz::Display)
