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

#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>

#include "sq2_ccv_roll_pitch_msgs/RollPitch.h"

namespace sq2_ccv_upper_body_controller{

  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder or sphere in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
   */
  class SQ2CCVUpperBodyController
      : public controller_interface::Controller<hardware_interface::PositionJointInterface>
  {
  public:
    SQ2CCVUpperBodyController();

    /**
     * \brief Initialize controller
     * \param hw            Velocity position 
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(hardware_interface::PositionJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);
    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);

  private:
    std::string name_;

    /// Hardware handles:
	std::vector<hardware_interface::JointHandle> roll_joints_;
	std::vector<hardware_interface::JointHandle> pitch_joints_;

    ros::Subscriber sub_command_;

	int roll_joints_size;
	int pitch_joints_size;

	/// params
	double roll_angle_absolute_value_limit_;
	double pitch_angle_absolute_value_limit_;
	std::vector<bool> roll_axes_reversed_;
	std::vector<bool> pitch_axes_reversed_;

    struct Commands
    {
      double roll;
      double pitch;
      ros::Time stamp;

      Commands() : roll(0.0), pitch(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;

  private:
    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
    void brake();

    /**
     * \brief command callback
     */
    void cmdCallback(const sq2_ccv_roll_pitch_msgs::RollPitch& command);

    /**
     * \brief Get the wheel names from a wheel param
     * \param [in]  controller_nh Controller node handler
     * \param [in]  joint_param   Param name
     * \param [out] joint_names   Vector with the whel names
     * \return true if the joint_param is available and the joint_names are
     *        retrieved successfully from the param server; false otherwise
     */
    bool getJointNames(ros::NodeHandle& controller_nh,
                       const std::string& joint_param,
                       std::vector<std::string>& joint_names);

    bool getAxesReverseds(ros::NodeHandle& controller_nh,
                          const std::string& axes_param,
                          std::vector<bool>& joint_axes);

  };

  PLUGINLIB_EXPORT_CLASS(sq2_ccv_upper_body_controller::SQ2CCVUpperBodyController, controller_interface::ControllerBase);
} // namespace sq2_ccv_upper_body_controller
