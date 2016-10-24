/*
 * Copyright 2013 Open Source Robotics Foundation
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
   Desc: GazeboRosSetJoint plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_set_joint.h>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosSetJoint::GazeboRosSetJoint()
{
  this->joint_msg_.data = 0;
  this->current_position_ = 0;
  this->current_speed_ = 0;
  this->delta_ = 0.00035; // radians. Move 0.02 degrees/iteration, or 20 degrees/sec at 1000 simulations/sec
  this->reversed_direction_ = 0; // default forward
  this->scale_ = 1;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSetJoint::~GazeboRosSetJoint()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSetJoint::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();
  
  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("jointName"))
  {
    ROS_FATAL("set_joint plugin missing <jointName>, cannot proceed");
    return;
  }
  else
    this->joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();

  this->joint_ = _model->GetJoint(this->joint_name_);
  if (!this->joint_)
  {
    ROS_FATAL("gazebo_ros_set_joint plugin error: joint named: %s does not exist\n",this->joint_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL("set_joint plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("actuationMode"))
  {
    ROS_FATAL("set_joint plugin missing <actuationMode>, cannot proceed");
    return;
  }
  else
    this->actuationMode_ = _sdf->GetElement("actuationMode")->Get<int>();

  if (_sdf->HasElement("reversed_direction"))
  {
    this->reversed_direction_ = _sdf->GetElement("reversed_direction")->Get<int>();
  }

  if (_sdf->HasElement("scale"))
  {
    this->scale_ = _sdf->GetElement("scale")->Get<double>();
  }

  std::cerr << "Loading gazebo_ros_set_joint with jointName " << this->joint_name_ 
            << " on topic " << this->topic_name_ 
            << " actuation mode: " << this->actuationMode_ 
            << " direction: " << this->reversed_direction_ 
            << std::endl;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64>(
    this->topic_name_,1,
    boost::bind( &GazeboRosSetJoint::UpdateCommandedPosition,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosSetJoint::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosSetJoint::UpdateChild, this));
  std::cerr << "Finished Load in gazebo_ros_set_joint\n";
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSetJoint::UpdateCommandedPosition(const std_msgs::Float64::ConstPtr& _msg)
{
  //this->lock_.lock();
  if (0 == this->reversed_direction_)
    this->joint_msg_.data = _msg->data * this->scale_;
  else
    this->joint_msg_.data = 0 - (_msg->data * this->scale_);
  //this->lock_.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSetJoint::UpdateChild()
{
  //this->lock_.lock();

  if (0 == this->actuationMode_)
  {
    // Check whether the joint needs to be moved to the requested position
    math::Angle angle = this->joint_->GetAngle(0);
    this->current_position_ = angle.Radian();

    if (fabs(this->current_position_ - this->joint_msg_.data) < 0.008)
    {
      return; // It hasn't drifted more than a half degree, and hasn't been commanded to move
    }
    else if (this->current_position_ < this->joint_msg_.data)
    {
      this->current_position_ += this->delta_;
    }
    else if (this->current_position_ > this->joint_msg_.data)
    {
      this->current_position_ -= this->delta_;
    }
    this->joint_->SetPosition(0, this->current_position_);
  }
  else if (1 == this->actuationMode_)
  {
    this->joint_->SetForce(0, this->joint_msg_.data);
  }
  else if (2 == this->actuationMode_)
  {
    double actual_speed = this->joint_->GetVelocity(0);
    this->current_speed_ += (this->delta_ * this->joint_msg_.data);
    if (fabs(actual_speed - this->current_speed_) > 0.5)
    {
      //std::cerr << "Changing drive velocity, actual: " << actual_speed << " requested: " << this->current_speed_ << std::endl;
      this->joint_->SetVelocity(0, this->current_speed_);
    }
  }

  //this->lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosSetJoint::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
