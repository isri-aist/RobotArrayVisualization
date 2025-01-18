#include "SingleRobotStateArrayDisplay.h"

#include <RBDyn/FK.h>
#include <RBDyn/parsers/urdf.h>

#include "ColorPropertySet.h"
#include "RobotStateArrayVisual.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <QFile>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/file_picker_property.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <class_loader/class_loader.hpp>

using namespace RobotArrayRvizPlugins;

enum DescriptionSource
{
  TOPIC_SOURCE, FILE_SOURCE
};

SingleRobotStateArrayDisplay::SingleRobotStateArrayDisplay()
{
  robot_description_source_property_ = new rviz_common::properties::EnumProperty(
      "Description Source", "Topic",
      "Source to get the robot description from.", this, SLOT(changedRobotDescriptionSource()));
  robot_description_source_property_->addOption("Topic", DescriptionSource::TOPIC_SOURCE);
  robot_description_source_property_->addOption("File", DescriptionSource::FILE_SOURCE);

  robot_description_property_ = new rviz_common::properties::RosTopicProperty(
      "Robot Description", "robot_description", rosidl_generator_traits::data_type<std_msgs::msg::String>(),
      "The name of the ROS parameter where the URDF for the robot is loaded",
      this, SLOT(changedRobotDescriptionTopic()), this);

  robot_description_file_property_ = new rviz_common::properties::FilePickerProperty(
    "Description File", "",
    "Path to the robot description.",
    this, SLOT(changedRobotDescriptionFile()));

  topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Robot State Array Topic", "robot_state_arr", rosidl_generator_traits::data_type<robot_array_msgs::msg::RobotStateArray>(),
      "The topic on which the robot_array_msgs::msg::RobotStateArray messages are received", 
      this, SLOT(changedRobotStateTopic()), this);

  check_name_property_ =
      new rviz_common::properties::BoolProperty("Check Name Consistency", false,
                             "Whether to devisualize robots with inconsistent names between URDF and message", this);

  robot_num_property_ =
      new rviz_common::properties::IntProperty("Max Robot Num", 5, "The maximum number of robots", this, SLOT(changedMaxRobotNum()), this);
  robot_num_property_->setMin(1);
  robot_num_property_->setMax(1000);

  robots_property_ = new rviz_common::properties::Property("Robots", QVariant(), "", this);

  color_style_property_ = new rviz_common::properties::EnumProperty("Robot Color Style", "Uniform", "The style of the robot color", this,
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

void SingleRobotStateArrayDisplay::load(const rviz_common::Config & config)
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
  // nh_ = std::make_shared<rclcpp::Node>("single_robot_state_array_display", "robot_array_rviz_plugins");
  nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    RCLCPP_WARN(nh_->get_logger(), "Unable to lock weak_ptr from DisplayContext in TrajectoryVisualization constructor");
    return;
  }
  robot_description_property_->initialize(ros_node_abstraction);
  topic_property_->initialize(ros_node_abstraction);

  Display::onInitialize();
  visual_ = std::make_shared<RobotStateArrayVisual>();
}

void SingleRobotStateArrayDisplay::onEnable()
{
  Display::onEnable();
  if(!robot_inited_)
  {
    robot_inited_ = true;
  }
  changedRobotDescriptionSource();
  changedRobotStateTopic();
  changedColorStyle();
}

void SingleRobotStateArrayDisplay::onDisable()
{
  robot_array_subscriber_.reset();
  robot_description_subscriber_.reset();
  Display::onDisable();
}

void SingleRobotStateArrayDisplay::fixedFrameChanged() {}

void SingleRobotStateArrayDisplay::robotStateArrayCallback(const robot_array_msgs::msg::RobotStateArray::SharedPtr msg)
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
  context_->getFrameManager()->getTransform(first_frame_id, frame_pos, frame_quat);

  scene_node_->setPosition(frame_pos);
  scene_node_->setOrientation(frame_quat);

  const auto & jointIndexByName = mb_.jointIndexByName();
  // assert(mb_.joint(0).name() == "Root");

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
    const robot_array_msgs::msg::RobotState & robot_state_msg = msg->robot_states[i];

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
    const geometry_msgs::msg::PoseStamped & root_pose_msg = robot_state_msg.root_pose;
    const geometry_msgs::msg::Point & root_pos_msg = root_pose_msg.pose.position;
    const geometry_msgs::msg::Quaternion & root_quat_msg = root_pose_msg.pose.orientation;
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
        RCLCPP_ERROR(nh_->get_logger(), "joint %s not found in the RBDyn robot model.", joint_name.c_str());
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
  setStatus(rviz_common::properties::StatusProperty::Ok, "SingleRobotStateArrayDisplay", QString::number(msg_num_) + " messages received");

  state_updated_ = true;
}

void SingleRobotStateArrayDisplay::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  urdf_content_ = msg->data;

  loadUrdfModel();
}

void SingleRobotStateArrayDisplay::loadUrdfModel()
{
  if (urdf_content_.empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to load a urdf model from robot_description.");
    return;
  }

  // set urdf_model
  std::unique_ptr<urdf::Model> urdf_model(new urdf::Model());

  if(!urdf_model->initString(urdf_content_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to init a urdf model from robot_description.");
    return;
  }

  int max_robot_num = robot_num_property_->getInt();

  // allocate robot model in visual_
  visual_->allocateRobotModel(max_robot_num, scene_node_, context_, robots_property_);

  // load robot model into visual_
  urdf_model_ = std::move(urdf_model);
  visual_->loadRobotModel(*urdf_model_);

  // load robot model into mb_ and mbc_list_
  const rbd::parsers::ParserResult & parse_res = rbd::parsers::from_urdf(urdf_content_, false);
  mb_ = parse_res.mb;
  name_ = parse_res.name;
  // parse_res.mbc is initialized by the forward kinematics calculation with the zero positions
  mbc_list_.resize(max_robot_num, parse_res.mbc);

  state_updated_ = true;
}

void SingleRobotStateArrayDisplay::changedRobotDescriptionSource()
{
  if (robot_description_source_property_->getOptionInt() == DescriptionSource::TOPIC_SOURCE) {
    robot_description_file_property_->setHidden(true);
    robot_description_property_->setHidden(false);
    changedRobotDescriptionTopic();
  } else if (robot_description_source_property_->getOptionInt() == DescriptionSource::FILE_SOURCE) {
    robot_description_property_->setHidden(true);
    robot_description_file_property_->setHidden(false);
    robot_description_subscriber_.reset();
    changedRobotDescriptionFile();
  }
}

void SingleRobotStateArrayDisplay::changedRobotDescriptionFile()
{
  if (robot_description_source_property_->getOptionInt() == DescriptionSource::FILE_SOURCE &&
    !robot_description_source_property_->getString().isEmpty())
  {
    std::string content;
    QFile urdf_file(QString::fromStdString(robot_description_file_property_->getString().toStdString()));
    if (urdf_file.open(QIODevice::ReadOnly)) {
      content = urdf_file.readAll().toStdString();
      urdf_file.close();
    }
    if (content.empty()) {
      setStatus(rviz_common::properties::StatusProperty::Error, "URDF", "URDF is empty");
      return;
    }
    if (content == urdf_content_) {
      return;
    }

    urdf_content_ = content;

    loadUrdfModel();

    setStatus(rviz_common::properties::StatusProperty::Ok, "URDF", "URDF is loaded");
  }
} 

void SingleRobotStateArrayDisplay::changedRobotStateTopic()
{
  robot_array_subscriber_.reset();

  setStatus(rviz_common::properties::StatusProperty::Warn, "SingleRobotStateArrayDisplay", "No message received");

  robot_num_ = 0;
  visual_->setVisible(robot_num_);
  
  robot_array_subscriber_ =
    nh_->create_subscription<robot_array_msgs::msg::RobotStateArray>(
        topic_property_->getStdString(), rclcpp::QoS(10), std::bind(&SingleRobotStateArrayDisplay::robotStateArrayCallback, this, std::placeholders::_1));

  state_updated_ = true;
}

void SingleRobotStateArrayDisplay::changedRobotDescriptionTopic()
{
  robot_description_subscriber_.reset();

  setStatus(rviz_common::properties::StatusProperty::Warn, "SingleRobotStateArrayDisplay", "No message received");
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.transient_local();

  robot_description_subscriber_ =
    nh_->create_subscription<std_msgs::msg::String>(
        robot_description_property_->getStdString(), qos, std::bind(&SingleRobotStateArrayDisplay::robotDescriptionCallback, this, std::placeholders::_1));
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
        color.getHSB(hsb_list[i][0], hsb_list[i][1], hsb_list[i][2]);
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

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RobotArrayRvizPlugins::SingleRobotStateArrayDisplay, rviz_common::Display)
