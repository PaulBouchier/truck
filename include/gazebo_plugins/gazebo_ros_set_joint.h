/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: A dynamic controller plugin that performs generic joint position setting interface.
 * Author: John Hsu
 * Date: 24 Sept 2008
 */

#ifndef GAZEBO_ROS_SET_JOINT_HH
#define GAZEBO_ROS_SET_JOINT_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float64.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosSetJoint Plugin XML Reference and Example

  \brief Ros Set Joint Position Plugin.
  
  This is a Plugin that collects data from a ROS topic and sets the position of a joint accordingly.

  Example Usage:
  \verbatim
      <gazebo>
        <plugin filename="libgazebo_ros_set_joint.so" name="gazebo_ros_set_joint">
          <jointName>the_joint</jointName>
          <topicName>joint_position</topicName>
        </plugin>
      </gazebo>
  \endverbatim
 
\{
*/

/**
           .
 
*/

class GazeboRosSetJoint : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboRosSetJoint();

  /// \brief Destructor
  public: virtual ~GazeboRosSetJoint();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateChild();

  /// \brief call back when a Joint set message is published
  /// \param[in] _msg The Incoming ROS message representing the new position to set.
  private: void UpdateCommandedPosition(const std_msgs::Float64::ConstPtr& _msg);

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Joint whose position is set
  private: physics::JointPtr joint_;
  /// \brief The current joint angle
  private: double current_position_;
  /// \brief The current speed
  private: double current_speed_;
  /// \brief The position delta to apply each simulator cycle if we aren't at the requested position
  private: double delta_;
  /// \brief The actuation direction.
  private: int reversed_direction_;
  /// \brief Multiply the requested setting by this
  private: double scale_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;

  /// \brief ROS joint setting topic name inputs
  private: std::string topic_name_;
  /// \brief The Joint this plugin is attached to, and whose position it will set
  private: std::string joint_name_;
  /// \brief actuation mode for the joint: 0 = position, 1 = force
  private: int actuationMode_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;
  /// \brief Container for the position that this plugin exerts on the joint.
  private: std_msgs::Float64 joint_msg_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosSetJoint);

/** \} */
/// @}
}
#endif
