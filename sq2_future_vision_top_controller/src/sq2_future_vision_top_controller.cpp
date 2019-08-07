/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <cmath>
#include <sq2_future_vision_top_controller/sq2_future_vision_top_controller.hpp>
#include <tf2/transform_datatypes.h>
#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) +
                   std::pow(vec1.z-vec2.z,2));
}

/*
* \brief Check that a link exists and has a geometry collision.
* \param link The link
* \return true if the link has a collision element with geometry 
*/
static bool hasCollisionGeometry(const urdf::LinkConstSharedPtr& link)
{
  if (!link)
  {
    ROS_ERROR("Link == NULL.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
    return false;
  }
  return true;
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr& link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_DEBUG_STREAM("Link " << link->name << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Check if the link is modeled as a sphere
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 */
static bool isSphere(const urdf::LinkConstSharedPtr& link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::SPHERE)
  {
    ROS_DEBUG_STREAM("Link " << link->name << " does not have sphere geometry");
    return false;
  }

  return true;
}

namespace sq2_ccv_upper_body_controller{

  SQ2CCVUpperBodyController::SQ2CCVUpperBodyController()
    : roll_angle_absolute_value_limit_(M_PI/6.0)
	, pitch_angle_absolute_value_limit_(M_PI/6.0)
    , roll_joints_size(0)
    , pitch_joints_size(0)
  {
  }

  bool SQ2CCVUpperBodyController::init(hardware_interface::PositionJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
	std::vector<std::string> roll_joint_names;
	std::vector<std::string> pitch_joint_names;
	getJointNames(controller_nh, "roll_joint", roll_joint_names);
	getJointNames(controller_nh, "pitch_joint", pitch_joint_names);
	getAxesReverseds(controller_nh, "roll_axis_reversed", roll_axes_reversed_);
	getAxesReverseds(controller_nh, "pitch_axis_reversed", pitch_axes_reversed_);
    
	if(roll_joint_names.size() > 0 && pitch_joint_names.size() > 0){
        roll_joints_size = roll_joint_names.size();
        pitch_joints_size = pitch_joint_names.size();
	}else{
		ROS_ERROR_STREAM_NAMED(name_, "roll or pitch has no joint");
		return false;
	}

	if(roll_joints_size != roll_axes_reversed_.size()){
		roll_axes_reversed_.clear();
		for(size_t i=0;i<roll_joints_size;i++){
            roll_axes_reversed_.push_back(false);
		}
	}
	if(pitch_joints_size != pitch_axes_reversed_.size()){
		pitch_axes_reversed_.clear();
		for(size_t i=0;i<pitch_joints_size;i++){
            pitch_axes_reversed_.push_back(false);
		}
	}

	roll_joints_.resize(roll_joints_size);
	pitch_joints_.resize(pitch_joints_size);

	for(size_t i = 0;i < roll_joints_size;i++){
        ROS_INFO_STREAM_NAMED(name_,
                              "Adding roll with joint name: " << roll_joint_names[i] << "\treversed: " << roll_axes_reversed_[i]);
		roll_joints_[i] = hw->getHandle(roll_joint_names[i]);
	}
	for(size_t i = 0;i < pitch_joints_size;i++){
        ROS_INFO_STREAM_NAMED(name_,
                              "Adding pitch with joint name: " << pitch_joint_names[i] << "\treversed: " << pitch_axes_reversed_[i]);
		pitch_joints_[i] = hw->getHandle(pitch_joint_names[i]);
	}

    sub_command_ = controller_nh.subscribe("roll_pitch", 1, &SQ2CCVUpperBodyController::cmdCallback, this);

    return true;
  }

  void SQ2CCVUpperBodyController::update(const ros::Time& time, const ros::Duration& period)
  {
    Commands curr_cmd = *(command_.readFromRT());
	double roll_angle = curr_cmd.roll;
	if(roll_angle > M_PI / 2.0){
		roll_angle -= M_PI;
	}else if(roll_angle < -M_PI / 2.0){
		roll_angle += M_PI;
	}
	for(size_t i=0;i<roll_joints_size;i++){
		double angle = roll_angle * (roll_axes_reversed_[i] ? -1 : 1);
		roll_joints_[i].setCommand(angle);
	}

	double pitch_angle = curr_cmd.pitch;
	if(pitch_angle > M_PI / 2.0){
		pitch_angle -= M_PI;
	}else if(pitch_angle < -M_PI / 2.0){
		pitch_angle += M_PI;
	}
	for(size_t i=0;i<pitch_joints_size;i++){
		double angle = pitch_angle * (pitch_axes_reversed_[i] ? -1 : 1);
		pitch_joints_[i].setCommand(angle);
	}
  }

  void SQ2CCVUpperBodyController::starting(const ros::Time& time)
  {
    brake();
  }

  void SQ2CCVUpperBodyController::stopping(const ros::Time& /*time*/)
  {
    brake();
  }

  void SQ2CCVUpperBodyController::brake()
  {
	for(size_t i=0;i<roll_joints_size;i++){
		roll_joints_[i].setCommand(0.0);
	}
	for(size_t i=0;i<pitch_joints_size;i++){
		pitch_joints_[i].setCommand(0.0);
	}
  }

  void SQ2CCVUpperBodyController::cmdCallback(const sq2_ccv_roll_pitch_msgs::RollPitch& command)
  {
    if (isRunning())
    {
      // check that we don't have multiple publishers on the command topic
	  Commands command_struct_;
      command_struct_.roll   = command.roll;
      command_struct_.pitch   = command.pitch;
      command_struct_.stamp = ros::Time::now();
      command_.writeFromNonRT (command_struct_);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  bool SQ2CCVUpperBodyController::getJointNames(ros::NodeHandle& controller_nh,
                              const std::string& joint_param,
                              std::vector<std::string>& joint_names)
  {
      XmlRpc::XmlRpcValue joint_list;
      if (!controller_nh.getParam(joint_param, joint_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve Joint param '" << joint_param << "'.");
        return false;
      }

      if (joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (joint_list.size() == 0)
        {
          ROS_ERROR_STREAM_NAMED(name_,
              "Joint param '" << joint_param << "' is an empty list");
          return false;
        }

        for (int i = 0; i < joint_list.size(); ++i)
        {
          if (joint_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_ERROR_STREAM_NAMED(name_,
                "Joint param '" << joint_param << "' #" << i <<
                " isn't a string.");
            return false;
          }
        }

        joint_names.resize(joint_list.size());
        for (int i = 0; i < joint_list.size(); ++i)
        {
          joint_names[i] = static_cast<std::string>(joint_list[i]);
        }
      }
      else if (joint_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        joint_names.push_back(joint_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Joint param '" << joint_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }

      return true;
  }

    bool SQ2CCVUpperBodyController::getAxesReverseds(ros::NodeHandle& controller_nh,
                          const std::string& axes_param,
                          std::vector<bool>& joint_axes)
  {
      XmlRpc::XmlRpcValue axes_list;
      if (!controller_nh.getParam(axes_param, axes_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve param '" << axes_param << "'.");
        return false;
      }

      if (axes_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (axes_list.size() == 0)
        {
          ROS_ERROR_STREAM_NAMED(name_,
              "Axes param '" << axes_param << "' is an empty list");
          return false;
        }

        for (int i = 0; i < axes_list.size(); ++i)
        {
          if (axes_list[i].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
          {
            ROS_ERROR_STREAM_NAMED(name_,
                "Axes param '" << axes_param << "' #" << i <<
                " isn't a string.");
            return false;
          }
        }

        joint_axes.resize(axes_list.size());
        for (int i = 0; i < axes_list.size(); ++i)
        {
          joint_axes[i] = static_cast<bool>(axes_list[i]);
        }
      }
      else if (axes_list.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      {
        joint_axes.push_back(axes_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Axes param '" << axes_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }

      return true;
  }
} // namespace sq2_ccv_upper_body_controller

